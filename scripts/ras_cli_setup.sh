#!/usr/bin/bash

load_experiment() {
    local exp_number=$1
    echo "Loading experiment $exp_number"
    ros2 service call /get_exepriment ras_interfaces/srv/LoadExp "{exepriment_id: '$exp_number', instruction_no: '', picked_object: ''}"
}


run_sim_robot() {
    echo "running sim robot"
    ros2 service call /test_experiment std_srvs/srv/SetBool "data: false"
}

run_real_robot() {
    echo "running real robot"
    ros2 action send_goal /execute_exp ras_interfaces/action/ExecuteExp {}
}


# register in bash configuration
ras_cli() {
    local command=$1
    shift
    case "$command" in
        load_experiment)
            load_experiment "$@"
            ;;
        run_sim_robot)
            run_sim_robot
            ;;
        run_real_robot)
            run_real_robot "$@"
            ;;
        -h|--help)
            echo "Usage: ras_cli <command> [options]"
            echo "Commands:"
            echo "  load_experiment <exp_number>   Load the specified experiment"
            echo "  run_sim_robot                Start the simulation robot for the specified experiment"
            echo "  run_real_robot               Start the real robot for the specified experiment"
            ;;
        *)
            echo "Unknown command: $command"
            echo "Use -h or --help for usage information."
            ;;
    esac
}

# Enable autocomplete for ras_cli
_complete_ras_cli() {
    local cur prev opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    opts="load_experiment run_sim_robot run_real_robot -h --help"

    if [[ ${cur} == -* ]]; then
        COMPREPLY=( $(compgen -W "${opts}" -- ${cur}) )
        return 0
    fi
}

complete -F _complete_ras_cli ras_cli
# Example usage
# ras_cli load_experiment 1