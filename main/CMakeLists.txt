idf_component_register(SRCS "app_main.c" "TASKS/task_encmot.c" "TASKS/task_graph.c" "TASKS/task_input.c"
                       INCLUDE_DIRS "." "TASKS"
                       REQUIRES Encoder PIDcontroller Motor EncMot)

