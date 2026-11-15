/* Offline regression for fixed DH offsets, Jacobians and gravity transforms. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <barrett/cdlbt/kinematics.h>
#include <barrett/cdlbt/calgrav.h>

static void near(double a, double b, double tol) {
    if (!(fabs(a-b) < tol)) {
        fprintf(stderr, "%.17g != %.17g (tolerance %g)\n", a, b, tol);
        assert(0);
    }
}

static void same_matrix(const gsl_matrix *a, const gsl_matrix *b) {
    assert(a->size1 == b->size1 && a->size2 == b->size2);
    for (size_t i=0; i<a->size1; ++i)
        for (size_t j=0; j<a->size2; ++j)
            near(gsl_matrix_get(a,i,j), gsl_matrix_get(b,i,j), 1e-11);
}

static void chain(int n) {
    config_t conf;
    config_init(&conf);
    char text[16384];
    int len=0;
    const double offsets[8]={.125,-.25,.375,-.547,.547,.5,.5,-.17};
    for (int variant=0; variant<2; ++variant) {
        len+=snprintf(text+len,sizeof(text)-len, "%s={world_to_base=((0,-1,0,.3),(1,0,0,-.4),(0,0,1,.7),(0,0,0,1)); moving=(", variant?"offset":"plain");
        for (int j=0; j<n; ++j) {
            len+=snprintf(text+len,sizeof(text)-len, "%s{alpha_pi=%.12f;a=%.12f;d=%.12f;",j?",":"",(j%3-1)*.5,.03*(j+1),.05*(j%2));
            if(variant) len+=snprintf(text+len,sizeof(text)-len,"theta_pi=%.12f;",offsets[j]);
            len+=snprintf(text+len,sizeof(text)-len,"}");
        }
        len+=snprintf(text+len,sizeof(text)-len,");toolplate={alpha_pi=.25;theta_pi=.31;a=.02;d=.1;};};");
    }
    len+=snprintf(text+len,sizeof(text)-len,"grav={mus=(");
    for(int j=0;j<n;++j) len+=snprintf(text+len,sizeof(text)-len,"%s(.13,-.21,.35)",j?",":"");
    snprintf(text+len,sizeof(text)-len,");};");
    assert(config_read_string(&conf,text));
    struct bt_kinematics *plain=NULL,*offset=NULL;
    struct bt_calgrav *grav=NULL;
    assert(!bt_kinematics_create(&plain,config_lookup(&conf,"plain"),n));
    assert(!bt_kinematics_create(&offset,config_lookup(&conf,"offset"),n));
    assert(!bt_calgrav_create(&grav,config_lookup(&conf,"grav"),n));
    gsl_vector *q=gsl_vector_alloc(n),*shifted=gsl_vector_alloc(n),*v=gsl_vector_alloc(n);
    gsl_vector *tau=gsl_vector_alloc(n),*expected=gsl_vector_alloc(n);
    /* Re-evaluate the same object, including a return to zero; no accumulation. */
    for(int pose=0;pose<12;++pose) {
        for(int j=0;j<n;++j) {
            double value=pose%3==0?0:.4*sin(pose+j);
            gsl_vector_set(q,j,value);
            gsl_vector_set(shifted,j,value+offsets[j]*acos(-1.0));
            gsl_vector_set(v,j,.1*cos(pose+j));
        }
        assert(!bt_kinematics_eval(plain,shifted,v));
        assert(!bt_kinematics_eval(offset,q,v));
        for(int j=0;j<n;++j) {
            same_matrix(plain->link[j]->trans_to_world,offset->link[j]->trans_to_world);
            near(gsl_vector_get(q,j),pose%3==0?0:.4*sin(pose+j),1e-12);
        }
        same_matrix(plain->toolplate->trans_to_world,offset->toolplate->trans_to_world);
        same_matrix(plain->tool_jacobian,offset->tool_jacobian);
        for(int j=0;j<3;++j)
            near(gsl_vector_get(plain->tool_velocity,j),gsl_vector_get(offset->tool_velocity,j),1e-12);
        assert(!bt_calgrav_eval(grav,plain,expected));
        assert(!bt_calgrav_eval(grav,offset,tau));
        for(int j=0;j<n;++j) near(gsl_vector_get(tau,j),gsl_vector_get(expected,j),1e-11);
        /* Check the position Jacobian against finite differences in q. */
        for(int j=0;j<n;++j) {
            double plus[3],minus[3],original=gsl_vector_get(q,j);
            gsl_vector_set(q,j,original+1e-6);bt_kinematics_eval(offset,q,NULL);
            for(int k=0;k<3;++k) plus[k]=gsl_matrix_get(offset->toolplate->trans_to_world,k,3);
            gsl_vector_set(q,j,original-1e-6);bt_kinematics_eval(offset,q,NULL);
            for(int k=0;k<3;++k) minus[k]=gsl_matrix_get(offset->toolplate->trans_to_world,k,3);
            gsl_vector_set(q,j,original);
            for(int k=0;k<3;++k) near((plus[k]-minus[k])/2e-6,gsl_matrix_get(plain->tool_jacobian,k,j),1e-8);
        }
    }
    gsl_vector_free(q);gsl_vector_free(shifted);gsl_vector_free(v);
    gsl_vector_free(tau);gsl_vector_free(expected);
    bt_calgrav_destroy(grav);bt_kinematics_destroy(plain);bt_kinematics_destroy(offset);
    config_destroy(&conf);
}

static void invalid_offsets(void) {
    const char *invalid[]={"true","\"oops\"","1e308"};
    for(int i=0;i<3;++i) {
        config_t conf;config_init(&conf);
        char text[512];
        snprintf(text,sizeof(text),"kin={moving=({alpha_pi=0;a=1;d=0;theta_pi=%s;});toolplate={alpha_pi=0;theta_pi=0;a=0;d=0;};};",invalid[i]);
        assert(config_read_string(&conf,text));
        struct bt_kinematics *kin=NULL;
        assert(bt_kinematics_create(&kin,config_lookup(&conf,"kin"),1)!=0);
        assert(kin==NULL);
        config_destroy(&conf);
    }
}

int main(void) {
    for(int n=1;n<=8;++n) chain(n);
    invalid_offsets();
    puts("PASS: 1-8 joints, fixed offsets, repeated evaluations, unchanged q, Jacobians, gravity and invalid offsets (no hardware).");
    return 0;
}
