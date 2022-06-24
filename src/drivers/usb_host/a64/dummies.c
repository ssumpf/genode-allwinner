
#include <linux/prandom.h>

DEFINE_PER_CPU(unsigned long, net_rand_noise);
EXPORT_PER_CPU_SYMBOL(net_rand_noise);


#include <linux/phy/phy.h>

int phy_exit(struct phy *phy)
{
	lx_emul_trace_and_stop(__func__);
}


int phy_init(struct phy *phy)
{
	lx_emul_trace_and_stop(__func__);
}


#include <linux/cpumask.h>

unsigned int cpumask_next(int n,const struct cpumask * srcp)
{
	lx_emul_trace(__func__);
	return n + 1;
}


#include <linux/sysfs.h>

int sysfs_create_dir_ns(struct kobject * kobj,const void * ns)
{
	lx_emul_trace(__func__);
	return 0;
}


#include <linux/random.h>

int __must_check get_random_bytes_arch(void * buf,int nbytes)
{
	printk("get_random_bytes_arch: leaving buffer unmodified\n");
	return 0;
}


#include <linux/fs.h>

int __register_chrdev(unsigned int major,unsigned int baseminor,unsigned int count,const char * name,const struct file_operations * fops)
{
	lx_emul_trace(__func__);
	return 0;
}


int register_chrdev_region(dev_t from,unsigned count,const char * name)
{
	lx_emul_trace(__func__);
	return 0;
}


#include <linux/cdev.h>

void cdev_init(struct cdev * cdev,const struct file_operations * fops)
{
	lx_emul_trace(__func__);
}


int cdev_add(struct cdev * p,dev_t dev,unsigned count)
{
	lx_emul_trace(__func__);
	return 0;
}


void cdev_del(struct cdev * p)
{
	lx_emul_trace(__func__);
}


#include <linux/proc_fs.h>

struct proc_dir_entry { int dummy; };

struct proc_dir_entry * proc_create_seq_private(const char * name,umode_t mode,struct proc_dir_entry * parent,const struct seq_operations * ops,unsigned int state_size,void * data)
{
	static struct proc_dir_entry ret;
	lx_emul_trace(__func__);
	return &ret;
}






