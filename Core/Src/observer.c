#include "observer.h"
#include "structs.h"
#include "user_config.h"
#include <stdio.h>

void dump_state(){

    if (!PHASE_ORDER ){
        printf("!PHASE_ORDER \r\n");
    }else{
        printf("RIGHT PHASE_ORDER \r\n");
    }
    printf("CURRENTS: \t a: %.5f \t b: %.5f \t c: %.5f \r\n", controller.foc_i_a, controller.foc_i_b, controller.foc_i_c);
    printf("DUTY CYCLE: \t dtc_u %.5f \t dtc_v %.5f \t dtc_w %.5f \r\n", controller.dtc_u, controller.dtc_v, controller.dtc_w);
    printf("DUTY CYCLE: \t theta_elec %.5f", controller.theta_elec);

}
    
