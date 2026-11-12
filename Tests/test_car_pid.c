#include "Car_PID.h"
#include "Car_Reference.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

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
    puts("advanced speed controller: all checks passed");
    return 0;
}
