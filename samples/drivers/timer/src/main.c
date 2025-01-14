#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/policy.h>
#include <am_mcu_apollo.h>
#include <am_util.h>

typedef struct
{
    uint32_t type;
    uintptr_t data;
} q_event_t;
K_MSGQ_DEFINE(event_queue, sizeof(q_event_t), 32, 4);
struct k_sem event_queue_dummy_sem;
struct k_timer timer1;
struct k_timer timer2;
struct k_timer timer3;
struct k_timer timer4;
struct k_timer timer5;
struct k_timer timer1000;
struct k_timer timer1001;
void timeout1_handler(struct k_timer *p)
{
    q_event_t ev = { 1, 100 };
    k_msgq_put(&event_queue, &ev, K_NO_WAIT);
    k_sem_give(&event_queue_dummy_sem);
    k_timer_start(&timer1, K_MSEC(100), K_NO_WAIT);
}
void timeout2_handler(struct k_timer *p)
{
    q_event_t ev = { 2, 200 };
    k_msgq_put(&event_queue, &ev, K_NO_WAIT);
    k_sem_give(&event_queue_dummy_sem);
    k_timer_start(&timer2, K_MSEC(200), K_NO_WAIT);
}
void timeout3_handler(struct k_timer *p)
{
    q_event_t ev = { 3, 300 };
    k_msgq_put(&event_queue, &ev, K_NO_WAIT);
    k_sem_give(&event_queue_dummy_sem);
    k_timer_start(&timer3, K_MSEC(300), K_NO_WAIT);
}
void timeout4_handler(struct k_timer *p)
{
    q_event_t ev = { 4, 400 };
    k_msgq_put(&event_queue, &ev, K_NO_WAIT);
    k_sem_give(&event_queue_dummy_sem);
    k_timer_start(&timer4, K_MSEC(400), K_NO_WAIT);
}
void timeout5_handler(struct k_timer *p)
{
    q_event_t ev = { 5, 500 };
    k_msgq_put(&event_queue, &ev, K_NO_WAIT);
    k_sem_give(&event_queue_dummy_sem);
    k_timer_start(&timer5, K_MSEC(500), K_NO_WAIT);
}
void timeout1000_handler(struct k_timer *p)
{
}
void timeout1001_handler(struct k_timer *p)
{
    k_timer_start(&timer1001, K_MSEC(1), K_NO_WAIT);
}
bool
z_arm_on_enter_cpu_idle(void)
{
    //_Static_assert(CONFIG_ARM_ON_ENTER_CPU_IDLE_HOOK != 0, "fw build without CONFIG_ARM_ON_ENTER_CPU_IDLE_HOOK");
    return false; // Prevent WFI sleep in Zephyr core
}
int
main(void)
{
    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES); // Disable WFI deep-sleep (PM subsystem)
    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES); // Disable WFI sleep (PM subsystem)
    k_timer_init(&timer1, timeout1_handler, NULL);
    k_timer_init(&timer2, timeout2_handler, NULL);
    k_timer_init(&timer3, timeout3_handler, NULL);
    k_timer_init(&timer4, timeout4_handler, NULL);
    k_timer_init(&timer5, timeout5_handler, NULL);
    k_timer_init(&timer1000, timeout1000_handler, NULL);
    k_timer_init(&timer1001, timeout1001_handler, NULL);
    k_timer_start(&timer1, K_MSEC(100), K_MSEC(100));
    k_timer_start(&timer2, K_MSEC(200), K_NO_WAIT);
    k_timer_start(&timer3, K_MSEC(300), K_NO_WAIT);
    k_timer_start(&timer4, K_MSEC(400), K_NO_WAIT);
    k_timer_start(&timer5, K_MSEC(500), K_NO_WAIT);
    k_timer_start(&timer1000, K_MSEC(1), K_MSEC(1));
    k_timer_start(&timer1001, K_MSEC(1), K_NO_WAIT);
    k_sem_init(&event_queue_dummy_sem, 1, 1);
    while (1)
    {
        q_event_t ev = { 0 };
        if (k_msgq_get(&event_queue, &ev, K_FOREVER) >= 0)
        {
            printk("q ev %u %u\n", (unsigned int)ev.type, (unsigned int)ev.data);
        }
    }
}
