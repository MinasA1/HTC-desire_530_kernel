/*
 * fs/f2fs/recovery.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/fs.h>
#include <linux/f2fs_fs.h>
#include "f2fs.h"
#include "node.h"
#include "segment.h"


static struct kmem_cache *fsync_entry_slab;

bool space_for_roll_forward(struct f2fs_sb_info *sbi)
{
	if (sbi->last_valid_block_count + sbi->alloc_valid_block_count
			> sbi->user_block_count)
		return false;
	return true;
}

static struct fsync_inode_entry *get_fsync_inode(struct list_head *head,
								nid_t ino)
{
	struct fsync_inode_entry *entry;

	list_for_each_entry(entry, head, list)
		if (entry->inode->i_ino == ino)
			return entry;

	return NULL;
}

static int recover_dentry(struct inode *inode, struct page *ipage)
{
	struct f2fs_inode *raw_inode = F2FS_INODE(ipage);
	nid_t pino = le32_to_cpu(raw_inode->i_pino);
	struct f2fs_dir_entry *de;
	struct qstr name;
	struct page *page;
	struct inode *dir, *einode;
	int err = 0;

	dir = f2fs_iget(inode->i_sb, pino);
	if (IS_ERR(dir)) {
		err = PTR_ERR(dir);
		goto out;
	}

	if (file_enc_name(inode)) {
		iput(dir);
		return 0;
	}

	name.len = le32_to_cpu(raw_inode->i_namelen);
	name.name = raw_inode->i_name;

	if (unlikely(name.len > F2FS_NAME_LEN)) {
		WARN_ON(1);
		err = -ENAMETOOLONG;
		goto out_err;
	}
retry:
	de = f2fs_find_entry(dir, &name, &page);
	if (de && inode->i_ino == le32_to_cpu(de->ino))
		goto out_unmap_put;

	if (de) {
		einode = f2fs_iget(inode->i_sb, le32_to_cpu(de->ino));
		if (IS_ERR(einode)) {
			WARN_ON(1);
			err = PTR_ERR(einode);
			if (err == -ENOENT)
				err = -EEXIST;
			goto out_unmap_put;
		}
		err = acquire_orphan_inode(F2FS_I_SB(inode));
		if (err) {
			iput(einode);
			goto out_unmap_put;
		}
		f2fs_delete_entry(de, page, dir, einode);
		iput(einode);
		goto retry;
	}
	err = __f2fs_add_link(dir, &name, inode, inode->i_ino, inode->i_mode);
	if (err)
		goto out_err;

	if (is_inode_flag_set(F2FS_I(dir), FI_DELAY_IPUT)) {
		iput(dir);
	} else {
		add_dirty_dir_inode(dir);
		set_inode_flag(F2FS_I(dir), FI_DELAY_IPUT);
	}

	goto out;

out_unmap_put:
	f2fs_dentry_kunmap(dir, page);
	f2fs_put_page(page, 0);
out_err:
	iput(dir);
out:
	f2fs_msg(inode->i_sb, KERN_NOTICE,
			"%s: ino = %x, name = %s, dir = %lx, err = %d",
			__func__, ino_of_node(ipage), raw_inode->i_name,
			IS_ERR(dir) ? 0 : dir->i_ino, err);
	return err;
}

static void recover_inode(struct inode *inode, struct page *page)
{
	struct f2fs_inode *raw = F2FS_INODE(page);
	char *name;

	inode->i_mode = le16_to_cpu(raw->i_mode);
	i_size_write(inode, le64_to_cpu(raw->i_size));
	inode->i_atime.tv_sec = le64_to_cpu(raw->i_mtime);
	inode->i_ctime.tv_sec = le64_to_cpu(raw->i_ctime);
	inode->i_mtime.tv_sec = le64_to_cpu(raw->i_mtime);
	inode->i_atime.tv_nsec = le32_to_cpu(raw->i_mtime_nsec);
	inode->i_ctime.tv_nsec = le32_to_cpu(raw->i_ctime_nsec);
	inode->i_mtime.tv_nsec = le32_to_cpu(raw->i_mtime_nsec);

	if (file_enc_name(inode))
		name = "<encrypted>";
	else
		name = F2FS_INODE(page)->i_name;

	f2fs_msg(inode->i_sb, KERN_NOTICE, "recover_inode: ino = %x, name = %s",
			ino_of_node(page), name);
}

static int find_fsync_dnodes(struct f2fs_sb_info *sbi, struct list_head *head)
{
	unsigned long long cp_ver = cur_cp_version(F2FS_CKPT(sbi));
	struct curseg_info *curseg;
	struct page *page = NULL;
	block_t blkaddr;
	int err = 0;

	
	curseg = CURSEG_I(sbi, CURSEG_WARM_NODE);
	blkaddr = NEXT_FREE_BLKADDR(sbi, curseg);

	ra_meta_pages(sbi, blkaddr, 1, META_POR, true);

	while (1) {
		struct fsync_inode_entry *entry;

		if (!is_valid_blkaddr(sbi, blkaddr, META_POR))
			return 0;

		page = get_tmp_page(sbi, blkaddr);

		if (cp_ver != cpver_of_node(page))
			break;

		if (!is_fsync_dnode(page))
			goto next;

		entry = get_fsync_inode(head, ino_of_node(page));
		if (!entry) {
			if (IS_INODE(page) && is_dent_dnode(page)) {
				err = recover_inode_page(sbi, page);
				if (err)
					break;
			}

			
			entry = kmem_cache_alloc(fsync_entry_slab, GFP_F2FS_ZERO);
			if (!entry) {
				err = -ENOMEM;
				break;
			}
			entry->inode = f2fs_iget(sbi->sb, ino_of_node(page));
			if (IS_ERR(entry->inode)) {
				err = PTR_ERR(entry->inode);
				kmem_cache_free(fsync_entry_slab, entry);
				if (err == -ENOENT) {
					err = 0;
					goto next;
				}
				break;
			}
			list_add_tail(&entry->list, head);
		}
		entry->blkaddr = blkaddr;

		if (IS_INODE(page)) {
			entry->last_inode = blkaddr;
			if (is_dent_dnode(page))
				entry->last_dentry = blkaddr;
		}
next:
		
		blkaddr = next_blkaddr_of_node(page);
		f2fs_put_page(page, 1);

		ra_meta_pages_cond(sbi, blkaddr);
	}
	f2fs_put_page(page, 1);
	return err;
}

static void destroy_fsync_dnodes(struct list_head *head)
{
	struct fsync_inode_entry *entry, *tmp;

	list_for_each_entry_safe(entry, tmp, head, list) {
		iput(entry->inode);
		list_del(&entry->list);
		kmem_cache_free(fsync_entry_slab, entry);
	}
}

static int check_index_in_prev_nodes(struct f2fs_sb_info *sbi,
			block_t blkaddr, struct dnode_of_data *dn)
{
	struct seg_entry *sentry;
	unsigned int segno = GET_SEGNO(sbi, blkaddr);
	unsigned short blkoff = GET_BLKOFF_FROM_SEG0(sbi, blkaddr);
	struct f2fs_summary_block *sum_node;
	struct f2fs_summary sum;
	struct page *sum_page, *node_page;
	struct dnode_of_data tdn = *dn;
	nid_t ino, nid;
	struct inode *inode;
	unsigned int offset;
	block_t bidx;
	int i;

	sentry = get_seg_entry(sbi, segno);
	if (!f2fs_test_bit(blkoff, sentry->cur_valid_map))
		return 0;

	
	for (i = CURSEG_WARM_DATA; i <= CURSEG_COLD_DATA; i++) {
		struct curseg_info *curseg = CURSEG_I(sbi, i);
		if (curseg->segno == segno) {
			sum = curseg->sum_blk->entries[blkoff];
			goto got_it;
		}
	}

	sum_page = get_sum_page(sbi, segno);
	sum_node = (struct f2fs_summary_block *)page_address(sum_page);
	sum = sum_node->entries[blkoff];
	f2fs_put_page(sum_page, 1);
got_it:
	
	nid = le32_to_cpu(sum.nid);
	if (dn->inode->i_ino == nid) {
		tdn.nid = nid;
		if (!dn->inode_page_locked)
			lock_page(dn->inode_page);
		tdn.node_page = dn->inode_page;
		tdn.ofs_in_node = le16_to_cpu(sum.ofs_in_node);
		goto truncate_out;
	} else if (dn->nid == nid) {
		tdn.ofs_in_node = le16_to_cpu(sum.ofs_in_node);
		goto truncate_out;
	}

	
	node_page = get_node_page(sbi, nid);
	if (IS_ERR(node_page))
		return PTR_ERR(node_page);

	offset = ofs_of_node(node_page);
	ino = ino_of_node(node_page);
	f2fs_put_page(node_page, 1);

	if (ino != dn->inode->i_ino) {
		
		inode = f2fs_iget(sbi->sb, ino);
		if (IS_ERR(inode))
			return PTR_ERR(inode);
	} else {
		inode = dn->inode;
	}

	bidx = start_bidx_of_node(offset, F2FS_I(inode)) +
			le16_to_cpu(sum.ofs_in_node);

	if (ino == dn->inode->i_ino && dn->inode_page_locked)
		unlock_page(dn->inode_page);

	set_new_dnode(&tdn, inode, NULL, NULL, 0);
	if (get_dnode_of_data(&tdn, bidx, LOOKUP_NODE))
		goto out;

	if (tdn.data_blkaddr == blkaddr)
		truncate_data_blocks_range(&tdn, 1);

	f2fs_put_dnode(&tdn);
out:
	if (ino != dn->inode->i_ino)
		iput(inode);
	else if (dn->inode_page_locked)
		lock_page(dn->inode_page);
	return 0;

truncate_out:
	if (datablock_addr(tdn.node_page, tdn.ofs_in_node) == blkaddr)
		truncate_data_blocks_range(&tdn, 1);
	if (dn->inode->i_ino == nid && !dn->inode_page_locked)
		unlock_page(dn->inode_page);
	return 0;
}

static int do_recover_data(struct f2fs_sb_info *sbi, struct inode *inode,
					struct page *page, block_t blkaddr)
{
	struct f2fs_inode_info *fi = F2FS_I(inode);
	unsigned int start, end;
	struct dnode_of_data dn;
	struct node_info ni;
	int err = 0, recovered = 0;

	
	if (IS_INODE(page)) {
		recover_inline_xattr(inode, page);
	} else if (f2fs_has_xattr_block(ofs_of_node(page))) {
		recover_xattr_data(inode, page, blkaddr);
		goto out;
	}

	
	if (recover_inline_data(inode, page))
		goto out;

	
	start = start_bidx_of_node(ofs_of_node(page), fi);
	end = start + ADDRS_PER_PAGE(page, fi);

	set_new_dnode(&dn, inode, NULL, NULL, 0);

	err = get_dnode_of_data(&dn, start, ALLOC_NODE);
	if (err)
		goto out;

	f2fs_wait_on_page_writeback(dn.node_page, NODE);

	get_node_info(sbi, dn.nid, &ni);
	f2fs_bug_on(sbi, ni.ino != ino_of_node(page));
	f2fs_bug_on(sbi, ofs_of_node(dn.node_page) != ofs_of_node(page));

	for (; start < end; start++, dn.ofs_in_node++) {
		block_t src, dest;

		src = datablock_addr(dn.node_page, dn.ofs_in_node);
		dest = datablock_addr(page, dn.ofs_in_node);

		
		if (src == dest)
			continue;

		
		if (dest == NULL_ADDR) {
			truncate_data_blocks_range(&dn, 1);
			continue;
		}

		if (dest == NEW_ADDR) {
			truncate_data_blocks_range(&dn, 1);
			err = reserve_new_block(&dn);
			f2fs_bug_on(sbi, err);
			continue;
		}

		
		if (is_valid_blkaddr(sbi, dest, META_POR)) {

			if (src == NULL_ADDR) {
				err = reserve_new_block(&dn);
				
				f2fs_bug_on(sbi, err);
			}

			
			err = check_index_in_prev_nodes(sbi, dest, &dn);
			if (err)
				goto err;

			
			f2fs_replace_block(sbi, &dn, src, dest,
							ni.version, false);
			recovered++;
		}
	}

	if (IS_INODE(dn.node_page))
		sync_inode_page(&dn);

	copy_node_footer(dn.node_page, page);
	fill_node_footer(dn.node_page, dn.nid, ni.ino,
					ofs_of_node(page), false);
	set_page_dirty(dn.node_page);
err:
	f2fs_put_dnode(&dn);
out:
	f2fs_msg(sbi->sb, KERN_NOTICE,
		"recover_data: ino = %lx, recovered = %d blocks, err = %d",
		inode->i_ino, recovered, err);
	return err;
}

static int recover_data(struct f2fs_sb_info *sbi,
				struct list_head *head, int type)
{
	unsigned long long cp_ver = cur_cp_version(F2FS_CKPT(sbi));
	struct curseg_info *curseg;
	struct page *page = NULL;
	int err = 0;
	block_t blkaddr;

	
	curseg = CURSEG_I(sbi, type);
	blkaddr = NEXT_FREE_BLKADDR(sbi, curseg);

	while (1) {
		struct fsync_inode_entry *entry;

		if (!is_valid_blkaddr(sbi, blkaddr, META_POR))
			break;

		ra_meta_pages_cond(sbi, blkaddr);

		page = get_tmp_page(sbi, blkaddr);

		if (cp_ver != cpver_of_node(page)) {
			f2fs_put_page(page, 1);
			break;
		}

		entry = get_fsync_inode(head, ino_of_node(page));
		if (!entry)
			goto next;
		if (entry->last_inode == blkaddr)
			recover_inode(entry->inode, page);
		if (entry->last_dentry == blkaddr) {
			err = recover_dentry(entry->inode, page);
			if (err) {
				f2fs_put_page(page, 1);
				break;
			}
		}
		err = do_recover_data(sbi, entry->inode, page, blkaddr);
		if (err) {
			f2fs_put_page(page, 1);
			break;
		}

		if (entry->blkaddr == blkaddr) {
			iput(entry->inode);
			list_del(&entry->list);
			kmem_cache_free(fsync_entry_slab, entry);
		}
next:
		
		blkaddr = next_blkaddr_of_node(page);
		f2fs_put_page(page, 1);
	}
	if (!err)
		allocate_new_segments(sbi);
	return err;
}

int recover_fsync_data(struct f2fs_sb_info *sbi)
{
	struct curseg_info *curseg = CURSEG_I(sbi, CURSEG_WARM_NODE);
	struct list_head inode_list;
	block_t blkaddr;
	int err;
	bool need_writecp = false;

	fsync_entry_slab = f2fs_kmem_cache_create("f2fs_fsync_inode_entry",
			sizeof(struct fsync_inode_entry));
	if (!fsync_entry_slab)
		return -ENOMEM;

	INIT_LIST_HEAD(&inode_list);

	
	mutex_lock(&sbi->cp_mutex);

	blkaddr = NEXT_FREE_BLKADDR(sbi, curseg);

	
	err = find_fsync_dnodes(sbi, &inode_list);
	if (err)
		goto out;

	if (list_empty(&inode_list))
		goto out;

	need_writecp = true;

	
	err = recover_data(sbi, &inode_list, CURSEG_WARM_NODE);
	if (!err)
		f2fs_bug_on(sbi, !list_empty(&inode_list));
out:
	destroy_fsync_dnodes(&inode_list);
	kmem_cache_destroy(fsync_entry_slab);

	
	truncate_inode_pages_range(META_MAPPING(sbi),
			(loff_t)MAIN_BLKADDR(sbi) << PAGE_CACHE_SHIFT, -1);

	if (err) {
		truncate_inode_pages(NODE_MAPPING(sbi), 0);
		truncate_inode_pages(META_MAPPING(sbi), 0);
	}

	clear_sbi_flag(sbi, SBI_POR_DOING);
	if (err) {
		bool invalidate = false;

		if (discard_next_dnode(sbi, blkaddr))
			invalidate = true;

		
		while (get_pages(sbi, F2FS_DIRTY_META))
			sync_meta_pages(sbi, META, LONG_MAX);

		
		if (invalidate)
			invalidate_mapping_pages(META_MAPPING(sbi),
							blkaddr, blkaddr);

		set_ckpt_flags(sbi->ckpt, CP_ERROR_FLAG);
		mutex_unlock(&sbi->cp_mutex);
	} else if (need_writecp) {
		struct cp_control cpc = {
			.reason = CP_RECOVERY,
		};
		mutex_unlock(&sbi->cp_mutex);
		write_checkpoint(sbi, &cpc);
	} else {
		mutex_unlock(&sbi->cp_mutex);
	}
	return err;
}
