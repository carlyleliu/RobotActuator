#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	printk("start main\n");

    while (1) {
		k_msleep(2);
	}

    return 1;
}
