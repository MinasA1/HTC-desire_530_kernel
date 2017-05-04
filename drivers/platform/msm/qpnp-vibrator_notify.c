/* drivers/platform/msm/qpnp-vibrator_notify.c
 *
 * Copyright (C) 2016 HTC Corporation.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 *
 */

#include <linux/qpnp_vibrator.h>

BLOCKING_NOTIFIER_HEAD(qpnp_vibrator_notifier_list);
int qpnp_vibrator_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&qpnp_vibrator_notifier_list, nb);
}
EXPORT_SYMBOL(qpnp_vibrator_register_notifier);

int qpnp_vibrator_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&qpnp_vibrator_notifier_list, nb);
}
EXPORT_SYMBOL(qpnp_vibrator_unregister_notifier);

int qpnp_vibrator_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&qpnp_vibrator_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(qpnp_vibrator_notifier_call_chain);
