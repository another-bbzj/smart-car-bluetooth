#include "Car_PID.h"
#include "Car_Reference.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned incremental_failures;

/* Keep running after a failed check so both saturation regressions are visible. */
static void check_incremental(float actual, float expected, const char *scenario)
{
    if (!isfinite(actual) || fabsf(actual - expected) > 0.02f)
    {
        fprintf(stderr, "%s: expected %.6g, got %.6g\n", scenario, expected, actual);
        incremental_failures++;
    }
}

static void test_incremental_saturation_recovery(void)
{
    Car_PidInc_Struct p;
    int sign;
    unsigned i;
    for (sign = -1; sign <= 1; sign += 2)
    {
        assert(Car_PidInc_init(&p, 100, 1, 0, 1000));
        for (i = 0; i < 4; i++)
            check_incremental(Car_PidInc_update(&p, sign * 100.0f, 0),
                              sign * 1000.0f, "hold saturated output");
        check_incremental(p.integral, 0, "saturation must reject integration");
        for (i = 0; i < 4; i++)
            check_incremental(Car_PidInc_update(&p, 0, 0), 0,
                              "zero demand after saturation");
        assert(!p.fault);
    }
}

static void test_incremental_rejected_integration(void)
{
    Car_PidInc_Struct p;
    int sign;
    for (sign = -1; sign <= 1; sign += 2)
    {
        assert(Car_PidInc_init(&p, 1, 2, 0, 100));
        check_incremental(Car_PidInc_update(&p, sign * 40.0f, 0),
                          sign * 40.0f, "rejected integral must not reach output");
        check_incremental(p.integral, 0, "rejected integral state");
        check_incremental(Car_PidInc_update(&p, 0, 0), 0,
                          "rejected integral must not leave residual output");
    }
}

static void test_incremental_integral_limit(void)
{
    Car_PidInc_Struct p;
    unsigned i;
    assert(Car_PidInc_init(&p, 0, 1, 0, 100));
    assert(Car_PidInc_configure(&p, 1, 0.02f, 0.5f, 0, 0, 0));
    check_incremental(Car_PidInc_update(&p, 30, 0), 30, "pure I first step");
    for (i = 0; i < 5; i++)
        check_incremental(Car_PidInc_update(&p, 30, 0), 50, "pure I cap");
    check_incremental(Car_PidInc_update(&p, 0, 0), 50, "pure I hold");
    check_incremental(Car_PidInc_update(&p, -20, 0), 30, "pure I unwind");
    check_incremental(Car_PidInc_update(&p, -30, 0), 0, "pure I return to zero");
}

static void test_incremental_reset_and_faults(void)
{
    Car_PidInc_Struct p, saved;
    assert(Car_PidInc_init(&p, 100, 1, 0, 1000));
    (void)Car_PidInc_update(&p, 100, 0);
    Car_PidInc_reset(&p);
    check_incremental(Car_PidInc_update(&p, 1, 0), 101, "reset clears all history");
    saved = p;
    assert(!Car_PidInc_configure(&p, 1, 0.02f, 1, NAN, 0, 0));
    assert(memcmp(&p, &saved, sizeof(p)) == 0);
    assert(Car_PidInc_configure(&p, 0.5f, 0, 1, 0, 0, 0));
    check_incremental(Car_PidInc_update(&p, 1, 0), 51, "configure clears history");
    assert(Car_PidInc_update(&p, NAN, 0) == 0 && p.fault);
    assert(Car_PidInc_update(&p, 1, 0) == 0 && p.fault);
    Car_PidInc_reset(&p);
    check_incremental(Car_PidInc_update(&p, 1, 0), 51, "reset recovers fault");
    assert(Car_PidInc_update_dt(&p, 1, 0, 0, 0) == 0 && p.fault);
    Car_PidInc_reset(&p);
    assert(Car_PidInc_update(&p, 0, INFINITY) == 0 && p.fault);
    assert(!Car_PidInc_init(&p, -1, 0, 0, 1000) && p.fault);
}

static void test_incremental_terms_and_timing(void)
{
    Car_PidInc_Struct p, q;
    float x = 0, y = 0;
    unsigned i;
    assert(Car_PidInc_init(&p, 0, 0.01f, 0, 1000));
    assert(Car_PidInc_init(&q, 0, 0.01f, 0, 1000));
    for (i = 0; i < 200; i++) x = Car_PidInc_update_dt(&p, 100, 0, 0, 0.005f);
    for (i = 0; i < 100; i++) y = Car_PidInc_update_dt(&q, 100, 0, 0, 0.010f);
    check_incremental(x, 200, "5 ms integration");
    check_incremental(y, 200, "10 ms integration");
    assert(Car_PidInc_init(&p, 0, 0, 0, 1000));
    assert(Car_PidInc_configure(&p, 1, 0.02f, 1, 10, 2, 0.1f));
    check_incremental(Car_PidInc_update_dt(&p, 150, 150, 50, 0.005f), 315,
                      "feedforward terms");
    check_incremental(Car_PidInc_update_dt(&p, -150, -150, -50, 0.005f), -315,
                      "reverse feedforward terms");
    assert(Car_PidInc_init(&p, 0, 0, 50, 1000));
    check_incremental(Car_PidInc_update(&p, 0, 0), 0, "initial derivative");
    check_incremental(Car_PidInc_update(&p, 100, 0), 0, "no setpoint derivative kick");
    check_incremental(Car_PidInc_update(&p, 100, 1), -10, "filtered measurement derivative");
}

static void test_incremental_periodic_no_drift(void)
{
    Car_PidInc_Struct p;
    float target[16] = {0}, measure[16] = {0}, acceleration[16] = {0};
    unsigned seed = 12345, i, cycle;
    assert(Car_PidInc_init(&p, 12, 0, 0, 1000));
    assert(Car_PidInc_configure(&p, 0.65f, 0.02f, 0.6f, 20, 1.2f, 0.1f));
    for (i = 0; i < 15; i++)
    {
        seed = seed * 1664525U + 1013904223U;
        target[i] = (float)((int)(seed % 361U) - 180);
        seed = seed * 1664525U + 1013904223U;
        measure[i] = (float)((int)(seed % 361U) - 180);
        seed = seed * 1664525U + 1013904223U;
        acceleration[i] = (float)((int)(seed % 401U) - 200);
    }
    /* Each cycle ends at zero P/FF; Ki=Kd=0 means the expected output is zero.
     * Repeating the same finite differences exposes systematic rounding drift. */
    for (cycle = 0; cycle < 100000; cycle++)
        for (i = 0; i < 16; i++)
            (void)Car_PidInc_update_dt(&p, target[i], measure[i], acceleration[i], 0.005f);
    check_incremental(p.last_output, 0, "periodic input must not accumulate output drift");
    assert(!p.fault);
}

static void test_incremental_matches_positional(void)
{
    Car_Pid_Struct pos;
    Car_PidInc_Struct inc;
    unsigned i, seed = 12345;
    assert(Car_Pid_init(&pos, 12, 0.3f, 8, 1000));
    assert(Car_PidInc_init(&inc, 12, 0.3f, 8, 1000));
    assert(Car_Pid_configure(&pos, 0.65f, 0.02f, 0.6f, 20, 1.2f, 0.1f));
    assert(Car_PidInc_configure(&inc, 0.65f, 0.02f, 0.6f, 20, 1.2f, 0.1f));
    for (i = 0; i < 20000; i++)
    {
        float target, measure, acceleration, dt, expected, actual;
        seed = seed * 1664525U + 1013904223U;
        target = (float)((int)(seed % 361U) - 180);
        seed = seed * 1664525U + 1013904223U;
        measure = (float)((int)(seed % 361U) - 180);
        seed = seed * 1664525U + 1013904223U;
        acceleration = (float)((int)(seed % 401U) - 200);
        dt = (1U + seed % 15U) * 0.001f;
        expected = Car_Pid_update_dt(&pos, target, measure, acceleration, dt);
        actual = Car_PidInc_update_dt(&inc, target, measure, acceleration, dt);
        check_incremental(actual, expected, "mixed terms with saturation and jitter");
        check_incremental(inc.integral, pos.integral, "conditional integration parity");
        assert(!pos.fault && !inc.fault);
        if (incremental_failures) break;
    }
}

int main(void)
{
    Car_Pid_Struct a,b;
    CarReference r;
    float x,y,old_ref[2]={0,0},old_acc[2]={0,0};
    float goal[2]={180,90};
    unsigned i,j;
    assert(Car_Pid_init(&a,0,0.01f,0,1000));
    assert(Car_Pid_init(&b,0,0.01f,0,1000));
    for(i=0;i<200;i++)x=Car_Pid_update_dt(&a,100,0,0,0.005f);
    for(i=0;i<100;i++)y=Car_Pid_update_dt(&b,100,0,0,0.010f);
    assert(fabsf(x-y)<0.01f && fabsf(x-200)<0.1f);
    assert(Car_Pid_init(&a,100,1,0,1000));
    for(i=0;i<1000;i++)assert(Car_Pid_update_dt(&a,100,0,0,0.005f)==1000);
    assert(fabsf(a.integral)<0.01f);
    assert(Car_Pid_update_dt(&a,0,0,0,0.005f)==0);
    assert(Car_Pid_init(&a,0,0,50,1000));
    assert(Car_Pid_configure(&a,1,0.02f,1,0,0,0));
    assert(Car_Pid_update_dt(&a,0,0,0,0.005f)==0);
    assert(Car_Pid_update_dt(&a,100,0,0,0.005f)==0); /* No setpoint D kick. */
    for(i=0;i<100;i++)assert(fabsf(Car_Pid_update_dt(&a,0,(i&1)?1:-1,0,0.005f))<30);
    assert(Car_Pid_init(&a,0,0,0,1000));
    assert(Car_Pid_configure(&a,1,0.02f,1,10,2,0.1f));
    assert(fabsf(Car_Pid_update_dt(&a,150,150,50,0.005f)-315)<0.01f);
    assert(!Car_Pid_configure(&a,1,0.02f,1,NAN,2,0));
    assert(a.kv==2);
    assert(Car_Pid_update_dt(&a,NAN,0,0,0.005f)==0 && a.fault);
    Car_Pid_reset(&a);
    assert(Car_Pid_update_dt(&a,10,0,0,0)==0 && a.fault);
    assert(!Car_Pid_init(&a,-1,0,0,1000));
    CarReference_Init(&r,180,200,800);
    for(i=0;i<1800;i++) {
        if(i==47){goal[0]=-180;goal[1]=100;}
        if(i==167){goal[0]=120;goal[1]=-160;}
        if(i==500){goal[0]=180;goal[1]=90;}
        CarReference_Update(&r,goal,0.005f);
        for(j=0;j<2;j++) {
            assert(fabsf(r.value[j])<=180.001f);
            assert(fabsf(r.acceleration[j])<=200.001f);
            assert(fabsf(r.acceleration[j]-old_acc[j])<=800*0.005f+0.05f);
            assert(fabsf(r.value[j]-old_ref[j])<=200*0.005f+0.01f);
            old_ref[j]=r.value[j];old_acc[j]=r.acceleration[j];
        }
    }
    assert(fabsf(r.value[0]-180)<0.01f && fabsf(r.value[1]-90)<0.01f);
    CarReference_Reset(&r);
    for(i=0;i<100;i++) {
        CarReference_Update(&r,goal,0.005f);
        assert(fabsf(r.value[0]-2*r.value[1])<0.01f); /* Shared progress preserves initial ratio. */
    }
    CarReference_Reset(&r);assert(r.value[0]==0 && r.acceleration[0]==0);
    /* Jitter and rapid retargeting: exercise the actual planner, not a model. */
    { unsigned seed=12345;
      CarReference_Reset(&r);old_ref[0]=old_ref[1]=old_acc[0]=old_acc[1]=0;
      for(i=0;i<20000;i++) {
        float dt;
        seed=seed*1664525U+1013904223U;dt=(1U+seed%15U)*0.001f;
        if(i%17U==0) {
            seed=seed*1664525U+1013904223U;goal[0]=(float)((int)(seed%361U)-180);
            seed=seed*1664525U+1013904223U;goal[1]=(float)((int)(seed%361U)-180);
        }
        CarReference_Update(&r,goal,dt);
        for(j=0;j<2;j++) {
            assert(fabsf(r.value[j])<=180.01f);
            assert(fabsf(r.acceleration[j])<=200.01f);
            assert(fabsf(r.acceleration[j]-old_acc[j])<=800*dt+0.1f);
            assert(fabsf(r.value[j]-old_ref[j])<=200*dt+0.01f);
            old_ref[j]=r.value[j];old_acc[j]=r.acceleration[j];
        }
      }
    }
    test_incremental_saturation_recovery();
    test_incremental_rejected_integration();
    test_incremental_integral_limit();
    test_incremental_reset_and_faults();
    test_incremental_terms_and_timing();
    test_incremental_periodic_no_drift();
    test_incremental_matches_positional();
    if (incremental_failures)
    {
        fprintf(stderr, "incremental PID: %u failed checks\n", incremental_failures);
        return 1;
    }
    puts("advanced speed controller: all checks passed (including 7 incremental PID scenarios)");
    return 0;
}
