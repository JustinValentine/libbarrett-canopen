/* Print the production C library's transforms for the offline viewer tests. */
#include <stdio.h>
#include <stdlib.h>
#include <barrett/cdlbt/kinematics.h>

static void matrix(const gsl_matrix *m) {
    printf("[");
    for (int i = 0; i < 4; ++i) {
        printf("%s[", i ? "," : "");
        for (int j = 0; j < 4; ++j)
            printf("%s%.17g", j ? "," : "", gsl_matrix_get(m, i, j));
        printf("]");
    }
    printf("]");
}

int main(int argc, char **argv) {
    if (argc < 3) return 2;
    config_t config;
    config_init(&config);
    if (!config_read_file(&config, argv[1])) {
        fprintf(stderr, "%s:%d: %s\n", argv[1], config_error_line(&config), config_error_text(&config));
        return 3;
    }
    struct bt_kinematics *kin = NULL;
    const int dof = argc - 2;
    if (bt_kinematics_create(&kin, config_lookup(&config, "robot.kinematics"), dof) || !kin) return 4;
    gsl_vector *q = gsl_vector_alloc(dof);
    for (int i = 0; i < dof; ++i) gsl_vector_set(q, i, strtod(argv[i + 2], NULL));
    if (bt_kinematics_eval(kin, q, NULL)) return 5;
    printf("{\"frames\":[");
    matrix(kin->base->trans_to_world);
    for (int i = 0; i < dof; ++i) {
        printf(",");
        matrix(kin->link[i]->trans_to_world);
    }
    printf("],\"tool\":");
    matrix(kin->toolplate->trans_to_world);
    printf("}\n");
    gsl_vector_free(q);
    bt_kinematics_destroy(kin);
    config_destroy(&config);
    return 0;
}
