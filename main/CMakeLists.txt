set(srcs
    "src/DHT.c")

set(includes "include")

idf_component_register(SRCS "rtos.c" "dht.c"
                    INCLUDE_DIRS ".")
