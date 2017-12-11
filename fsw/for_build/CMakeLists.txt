
add_airliner_app_def(sim
    FILE SIM
    SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_app.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_app.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_config_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_events.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_msg.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_tbldefs.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_version.h
   
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/
        ${PROJECT_SOURCE_DIR}/tools/mavlink/common/

    USER_DOCS   ${CMAKE_CURRENT_SOURCE_DIR}/user_doxy.in
    DESIGN_DOCS ${CMAKE_CURRENT_SOURCE_DIR}/detail_doxy.in
)

add_airliner_app_def(simlib
    FILE SIMLIB
    SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../public_inc/simlib.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/simlib_main.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_version.h
   
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/

    # This application has a public include header.  Add it so all the other applications can access it.
    PUBLIC_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/../public_inc/

    USER_DOCS   ${CMAKE_CURRENT_SOURCE_DIR}/lib_user_doxy.in
    DESIGN_DOCS ${CMAKE_CURRENT_SOURCE_DIR}/lib_detail_doxy.in
)

add_airliner_app_unit_test(sim-ut
    UTASSERT
    SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/sim_app_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/sim_app_test.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/sim_cmds_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/sim_cmds_test.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/sim_config_tbl_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/sim_config_tbl_test.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/sim_testrunner.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/sim_test_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/sim_test_utils.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_app.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_app.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_config_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_events.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_msg.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_tbldefs.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim_version.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../public_inc/simlib.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/simlib_main.c
        
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/
        ${PROJECT_SOURCE_DIR}/tools/mavlink/common/
        
    VALGRIND_SUPPRESSION_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../unit_test/valgrind.supp
)
