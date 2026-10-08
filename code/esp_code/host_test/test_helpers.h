/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Small helpers shared by all the host tests.
 *
 * Functions:
 *   - TEST_ASSERT_NO_CRASH(code, msg): runs code and turns a crash
 *     (SIGSEGV, SIGFPE, SIGBUS) into a normal test failure, so one crash
 *     does not stop the rest of the tests.
 *   - poison_stack(): fills the stack with garbage, to find variables
 *     that are used without being initialised.
 *   - DEG / RAD: angle conversions.
 */
#pragma once
#include <setjmp.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include "unity.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG(rad) ((rad) * 180.0 / M_PI)
#define RAD(deg) ((deg) * M_PI / 180.0)

static sigjmp_buf            th_crash_jmp;
static volatile sig_atomic_t th_crash_sig;

static void th_crash_handler(int sig) {
    th_crash_sig = sig;
    siglongjmp(th_crash_jmp, 1);
}

#define TEST_ASSERT_NO_CRASH(code, msg) do {                               \
        struct sigaction th_sa, th_old_segv, th_old_fpe, th_old_bus;       \
        memset(&th_sa, 0, sizeof th_sa);                                   \
        th_sa.sa_handler = th_crash_handler;                               \
        sigemptyset(&th_sa.sa_mask);                                       \
        th_sa.sa_flags = SA_NODEFER;                                       \
        sigaction(SIGSEGV, &th_sa, &th_old_segv);                          \
        sigaction(SIGFPE,  &th_sa, &th_old_fpe);                           \
        sigaction(SIGBUS,  &th_sa, &th_old_bus);                           \
        th_crash_sig = 0;                                                  \
        if (sigsetjmp(th_crash_jmp, 1) == 0) { code; }                     \
        sigaction(SIGSEGV, &th_old_segv, NULL);                            \
        sigaction(SIGFPE,  &th_old_fpe,  NULL);                            \
        sigaction(SIGBUS,  &th_old_bus,  NULL);                            \
        if (th_crash_sig) TEST_FAIL_MESSAGE(msg);                          \
    } while (0)

/* Fill the stack below the caller with a recognisable garbage value. */
__attribute__((noinline)) static void poison_stack(void) {
    volatile unsigned char junk[16384];
    memset((void *)junk, 0x5A, sizeof junk);
}
