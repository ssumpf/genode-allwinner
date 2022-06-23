
#include <linux/prandom.h>

DEFINE_PER_CPU(unsigned long, net_rand_noise);
EXPORT_PER_CPU_SYMBOL(net_rand_noise);


#include <linux/phy/phy.h>

int phy_exit(struct phy *phy)
{
	lx_emul_trace_and_stop(__func__);
}


#include <linux/phy/phy.h>

int phy_init(struct phy *phy)
{
	lx_emul_trace_and_stop(__func__);
}


