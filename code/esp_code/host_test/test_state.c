/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Unit tests for state.c (host, no hardware). They check that every
 *   setter saves its value, that every getter returns it, that the
 *   setters do not change other fields, and that every access uses the
 *   critical section and releases it (thread safety).
 *
 *   The .c file is included directly. Another version can be tested with:
 *       make run T=state MODULE_SRC=path/to/state.c
 *
 * Functions:
 *   - setUp() / tearDown(): reset the mocks and the state before each test.
 *   - test_init_*: state_init().
 *   - test_set_get_*: setters and getters.
 *   - test_lock_*: use of the critical section.
 *   - main(): runs all the tests.
 */
#include "unity.h"
#include "test_helpers.h"
#include "mocks/mock_reset.h"

#ifndef MODULE_SRC
#define MODULE_SRC "../components/global_data/state/state.c"
#endif
#include MODULE_SRC

#define EPS 1e-6f

/* Fill every field with a different value. */
static void fill_all(void) {
    set_position(1.0f, 2.0f);
    set_h(3.0f);
    set_orientation(0.1f, 0.2f, 0.3f, 0.9f);
    set_vel_lin(4.0f, 5.0f, 6.0f);
    set_vel_ang(7.0f, 8.0f, 9.0f);
    set_acc_lin(10.0f, 11.0f, 12.0f);
    set_time_imu(123456789LL);
    set_time_height(987654321LL);
    set_sm_state(HOVERING);
}

void setUp(void) {
    mock_reset_all();
    state_init();
    mock_reset_all();
}

void tearDown(void) {
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, mock_critical_depth, "a critical section was not released");
}

/* ------------------------------------------------------------------ */
/*                               INIT                                 */
/* ------------------------------------------------------------------ */
void test_init_resets_every_field(void) {
    fill_all();
    state_init();
    state_t s;
    get_global_state(&s);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.pos.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.pos.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.pos.z);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.vel_lin.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.vel_ang.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.acc_lin.z);
    TEST_ASSERT_EQUAL_INT64(0, s.t_stamp_imu);
    TEST_ASSERT_EQUAL_INT64(0, s.t_stamp_h);
}

void test_init_orientation_is_the_identity_quaternion(void) {
    fill_all();
    state_init();
    quat_t q;
    get_orientation(&q);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.z);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, q.w);
}

void test_init_state_machine_is_INIT(void) {
    set_sm_state(ERROR);
    state_init();
    sm_states_t sm;
    get_sm_state(&sm);
    TEST_ASSERT_EQUAL_INT(INIT, sm);
}

void test_init_states_have_the_expected_values(void) {
    /* Other modules (and the logs) use these numbers. */
    TEST_ASSERT_EQUAL_INT(0, INIT);
    TEST_ASSERT_EQUAL_INT(1, CHECKING);
    TEST_ASSERT_EQUAL_INT(2, ARMING);
    TEST_ASSERT_EQUAL_INT(3, TAKING_OFF);
    TEST_ASSERT_EQUAL_INT(4, HOVERING);
    TEST_ASSERT_EQUAL_INT(5, EXTERNAL_CONTROL);
    TEST_ASSERT_EQUAL_INT(6, LANDING);
    TEST_ASSERT_EQUAL_INT(7, DISARMING);
    TEST_ASSERT_EQUAL_INT(8, ERROR);
}

/* ------------------------------------------------------------------ */
/*                          SETTERS / GETTERS                         */
/* ------------------------------------------------------------------ */
void test_set_get_position_saves_x_and_y(void) {
    set_position(1.5f, -2.5f);
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.5f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -2.5f, p.y);
}

void test_set_get_position_does_not_change_the_height(void) {
    set_h(4.0f);
    set_position(1.0f, 1.0f);
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 4.0f, p.z);
}

void test_set_get_height_does_not_change_x_and_y(void) {
    set_position(1.0f, 2.0f);
    set_h(7.5f);
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, p.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 7.5f, p.z);
}

void test_set_get_orientation(void) {
    set_orientation(0.1f, -0.2f, 0.3f, 0.927f);
    quat_t q;
    get_orientation(&q);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.1f, q.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -0.2f, q.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.3f, q.z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.927f, q.w);
}

void test_set_get_linear_velocity(void) {
    set_vel_lin(1.0f, -2.0f, 3.0f);
    vec3_t v;
    get_vel_lin(&v);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, v.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -2.0f, v.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, v.z);
}

void test_set_get_angular_velocity(void) {
    set_vel_ang(-10.0f, 20.0f, -30.0f);
    vec3_t w;
    get_vel_ang(&w);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -10.0f, w.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 20.0f, w.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -30.0f, w.z);
}

void test_set_get_linear_acceleration(void) {
    set_acc_lin(0.0f, 0.5f, 1.0f);
    vec3_t a;
    get_acc_lin(&a);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, a.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.5f, a.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, a.z);
}

void test_set_get_timestamps_keep_64_bits(void) {
    const int64_t big = 5000000000000LL;   /* > 2^32, 5000 s in ns */
    set_time_imu(big);
    set_time_height(big + 1);
    int64_t ti, th;
    get_time_imu(&ti);
    get_time_height(&th);
    TEST_ASSERT_EQUAL_INT64(big, ti);
    TEST_ASSERT_EQUAL_INT64(big + 1, th);
}

void test_set_get_imu_and_height_timestamps_are_independent(void) {
    set_time_imu(10);
    set_time_height(20);
    int64_t ti, th;
    get_time_imu(&ti);
    get_time_height(&th);
    TEST_ASSERT_EQUAL_INT64(10, ti);
    TEST_ASSERT_EQUAL_INT64(20, th);
}

void test_set_get_state_machine_every_state(void) {
    for (int s = INIT; s <= ERROR; s++) {
        set_sm_state((sm_states_t)s);
        sm_states_t out;
        get_sm_state(&out);
        TEST_ASSERT_EQUAL_INT(s, out);
    }
}

void test_set_get_vectors_are_independent(void) {
    set_vel_lin(1.0f, 1.0f, 1.0f);
    set_vel_ang(2.0f, 2.0f, 2.0f);
    set_acc_lin(3.0f, 3.0f, 3.0f);
    vec3_t v, w, a;
    get_vel_lin(&v);
    get_vel_ang(&w);
    get_acc_lin(&a);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, v.z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, w.z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, a.z);
}

void test_set_get_global_state_copies_every_field(void) {
    fill_all();
    state_t s;
    get_global_state(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, s.pos.x);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, s.pos.y);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, s.pos.z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.9f, s.q.w);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 6.0f, s.vel_lin.z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 9.0f, s.vel_ang.z);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 12.0f, s.acc_lin.z);
    TEST_ASSERT_EQUAL_INT64(123456789LL, s.t_stamp_imu);
    TEST_ASSERT_EQUAL_INT64(987654321LL, s.t_stamp_h);
}

void test_set_get_global_state_is_a_copy(void) {
    set_h(1.0f);
    state_t s;
    get_global_state(&s);
    s.pos.z = 99.0f;              /* changing the copy ... */
    vec3_t p;
    get_position(&p);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, p.z);   /* ... does not change the state */
}

/* ------------------------------------------------------------------ */
/*                         CRITICAL SECTION                           */
/* ------------------------------------------------------------------ */
typedef void (*access_fn)(void);
static vec3_t  tmp_v;
static quat_t  tmp_q;
static int64_t tmp_t;
static sm_states_t tmp_sm;
static state_t tmp_s;
static void a1(void)  { set_position(1, 2); }
static void a2(void)  { set_h(1); }
static void a3(void)  { set_orientation(0, 0, 0, 1); }
static void a4(void)  { set_vel_lin(1, 2, 3); }
static void a5(void)  { set_vel_ang(1, 2, 3); }
static void a6(void)  { set_acc_lin(1, 2, 3); }
static void a7(void)  { set_time_imu(1); }
static void a8(void)  { set_time_height(1); }
static void a9(void)  { set_sm_state(ARMING); }
static void a10(void) { get_position(&tmp_v); }
static void a11(void) { get_orientation(&tmp_q); }
static void a12(void) { get_vel_lin(&tmp_v); }
static void a13(void) { get_vel_ang(&tmp_v); }
static void a14(void) { get_acc_lin(&tmp_v); }
static void a15(void) { get_time_imu(&tmp_t); }
static void a16(void) { get_time_height(&tmp_t); }
static void a17(void) { get_sm_state(&tmp_sm); }
static void a18(void) { get_global_state(&tmp_s); }
static void a19(void) { state_init(); }

static const access_fn ACCESS[] = { a1, a2, a3, a4, a5, a6, a7, a8, a9, a10,
                                    a11, a12, a13, a14, a15, a16, a17, a18, a19 };

void test_lock_every_function_uses_the_critical_section(void) {
    for (size_t i = 0; i < sizeof ACCESS / sizeof ACCESS[0]; i++) {
        int before = mock_critical_enter_count;
        ACCESS[i]();
        char msg[64];
        snprintf(msg, sizeof msg, "access function %zu does not use the lock", i + 1);
        TEST_ASSERT_TRUE_MESSAGE(mock_critical_enter_count > before, msg);
    }
}

void test_lock_every_function_releases_the_critical_section(void) {
    for (size_t i = 0; i < sizeof ACCESS / sizeof ACCESS[0]; i++) {
        ACCESS[i]();
        char msg[64];
        snprintf(msg, sizeof msg, "access function %zu does not release the lock", i + 1);
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, mock_critical_depth, msg);
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, s_lock.locked, msg);
    }
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_resets_every_field);
    RUN_TEST(test_init_orientation_is_the_identity_quaternion);
    RUN_TEST(test_init_state_machine_is_INIT);
    RUN_TEST(test_init_states_have_the_expected_values);

    RUN_TEST(test_set_get_position_saves_x_and_y);
    RUN_TEST(test_set_get_position_does_not_change_the_height);
    RUN_TEST(test_set_get_height_does_not_change_x_and_y);
    RUN_TEST(test_set_get_orientation);
    RUN_TEST(test_set_get_linear_velocity);
    RUN_TEST(test_set_get_angular_velocity);
    RUN_TEST(test_set_get_linear_acceleration);
    RUN_TEST(test_set_get_timestamps_keep_64_bits);
    RUN_TEST(test_set_get_imu_and_height_timestamps_are_independent);
    RUN_TEST(test_set_get_state_machine_every_state);
    RUN_TEST(test_set_get_vectors_are_independent);
    RUN_TEST(test_set_get_global_state_copies_every_field);
    RUN_TEST(test_set_get_global_state_is_a_copy);

    RUN_TEST(test_lock_every_function_uses_the_critical_section);
    RUN_TEST(test_lock_every_function_releases_the_critical_section);

    return UNITY_END();
}
