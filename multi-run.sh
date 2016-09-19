#!/bin/bash
set -e

function recompile() {
  cur_dir=$PWD
  cd "${REVOLVE_HOME}/revolve/build"
  echo '-> BUILDING REVOLVE'
  make
  cd "${REVOLVE_HOME}/tol-revolve/build"
  echo '-> BUILDING TOL-REVOLVE'
  make
  cd $cur_dir
}

function run_experiments() {
  # activate virtual environment
  if [ ! -z "${VENV_ACTIVATE}" ]; then
    source ${VENV_ACTIVATE}
  fi

  # search gazebo library in other path
  if [ ! -z "${GAZEBO_LIB_PATH}" ]; then
    export LD_LIBRARY_PATH="${GAZEBO_LIB_PATH}:${LD_LIBRARY_PATH}"
  fi

  #export NEAT_POP_SIZE=10
  export SUPG_MAX_EVALUATIONS=1000
  export SUPG_FREQUENCY_RATE=30
  export SUPG_CYCLE_LENGTH=5
  #export NEAT_SEARCH_TYPE="BLENDED"
  ROBOT_NAME="snake_9"

  mkdir -p "experiment2/$ROBOT_NAME"
  set -x

  for search_type in "PHASED" "COMPLEXIFY" "BLENDED"
  do for pop_size in 10 25
    do for i in $(seq 1 10)
      do
        LOGFILE="experiment2/${ROBOT_NAME}/run-${ROBOT_NAME}-${search_type}-${pop_size}-${i}.log"
        if [ -f ${LOGFILE} ]
        then
          echo "logfile ${LOGFILE} already existing, skipping"
        else
          #echo "NEAT_POP_SIZE=$pop_size ./start.py |tee ${LOGFILE}"
          NEAT_SEARCH_TYPE=${search_type} NEAT_POP_SIZE=${pop_size} ./start.py |tee "${LOGFILE}"
        fi
      done
    done
  done
}

function main() {
  if [ -z "${REVOLVE_HOME}" ]; then
    echo "REVOLVE_HOME variable not set, exiting now"
    return 1
  else
    echo "Using project home: ${REVOLVE_HOME}"
  fi

  if [ -z "${GAZEBO_LIB_PATH}" ]; then
    echo "GAZEBO_LIB_PATH variable not set"
    echo "  you can use this variable if you installed gazebo in a non standard directory"
  else
    echo "Using gazebo library path: ${GAZEBO_LIB_PATH}"
  fi

  if [ -z "${VENV_ACTIVATE}" ]; then
    echo "VENV_ACTIVATE variable not set"
    echo "  you can use this variable if you installed gazebo and revolve python libraries in a python virtual environment"
  else
    echo "Using python venv: ${VENV_ACTIVATE}"
  fi

  recompile
  run_experiments
}

main
