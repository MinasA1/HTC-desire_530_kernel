#ifndef __LINUX_QPNP_VIBRATOR_H
#define __LINUX_QPNP_VIBRATOR_H

#include <linux/notifier.h>


extern int qpnp_vibrator_register_notifier(struct notifier_block *nb);
extern int qpnp_vibrator_unregister_notifier(struct notifier_block *nb);
extern int qpnp_vibrator_notifier_call_chain(unsigned long val, void *v);

#endif
