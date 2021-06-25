# - Try to find PNC
#  Once done this will define
#  PNC_FOUND - System has Gurobi
#  PNC_INCLUDE_DIRS - The Gurobi include directories
#  PNC_LIBRARIES - The libraries needed to use Gurobi

find_path(PNC_INCLUDE_DIR
          NAMES Configuration.h
          PATHS "/usr/local/include/PnC"
          )
find_library(PNC_UTILS_LIBRARY
             NAMES myUtils
             PATHS "/usr/local/lib/"
             )
find_library(PNC_WBC_LIBRARY
             NAMES myWBC
             PATHS "/usr/local/lib/"
             )

find_library(PNC_MPC_LIBRARY
             NAMES myMPC
             PATHS "/usr/local/lib/"
             )

find_library(PNC_OSQP_LIBRARY
             NAMES myOSQP
             PATHS "/usr/local/lib/"
             )

find_library(PNC_GOLDFARB_LIBRARY
             NAMES myGoldfarb
             PATHS "/usr/local/lib/"
             )

find_library(PNC_FILTERS_LIBRARY
             NAMES myFilters
             PATHS "/usr/local/lib/"
             )

find_library(PNC_ROBOTSYSTEM_LIBRARY
             NAMES myRobotSystem
             PATHS "/usr/local/lib/"
             )

find_library(PNC_TM_LIBRARY
             NAMES myTrajectoryManager
             PATHS "/usr/local/lib/"
             )

find_library(PNC_YAML_LIBRARY
             NAMES myYaml
             PATHS "/usr/local/lib/"
             )

find_library(A1_PNC_LIBRARY
             NAMES A1PnC
             PATHS "/usr/local/lib/"
             )

include(FindPackageHandleStandardArgs)

if(PNC_INCLUDE_DIR)
    set(PNC_INCLUDE_DIRS "${PNC_INCLUDE_DIR}" )
    set(PNC_LIBRARIES  ${A1_PNC_LIBRARY}
                       ${PNC_UTILS_LIBRARY}
                       ${PNC_WBC_LIBRARY}
                       ${PNC_MPC_LIBRARY}
                       ${PNC_GOLDFARB_LIBRARY}
                       ${PNC_FILTERS_LIBRARY}
                       ${PNC_ROBOTSYSTEM_LIBRARY}
                       ${PNC_TM_LIBRARY}
                       ${PNC_YAML_LIBRARY}
                       ${PNC_OSQP_LIBRARY} )
    set(PNC_FOUND TRUE)
    message("-- Found PnC: TRUE")
    message("-- PNC_LIBRARIES")
    message(${PNC_LIBRARIES})
else()
    message("-- Found PnC: FALSE, Build without PnC")
endif()

mark_as_advanced( PNC_INCLUDE_DIR
                  A1_PNC_LIBRARY
                  PNC_UTILS_LIBRARY
                  PNC_WBC_LIBRARY
                  PNC_MPC_LIBRARY
                  PNC_GOLDFARB_LIBRARY
                  PNC_FILTERS_LIBRARY
                  PNC_ROBOTSYSTEM_LIBRARY
                  PNC_TM_LIBRARY
                  PNC_YAML_LIBRARY
                  PNC_OSQP_LIBRARY)
