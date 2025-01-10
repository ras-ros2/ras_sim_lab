#!/usr/bin/bash

load_experiment() {
    local exp_number=$1
    if [ -z "$exp_number" ]; then
        echo "Please provide a valid experiment number like"
        echo "ras_cli load_experiment 1"
        return 1
    fi
    echo "Loading experiment $exp_number"
    ros2 service call /get_exepriment ras_interfaces/srv/LoadExp "{exepriment_id: '$exp_number', instruction_no: '', picked_object: ''}"
}

run_sim_robot() {
    echo "Running simulation robot"
    ros2 service call /test_experiment std_srvs/srv/SetBool "data: false"
}

run_real_robot() {
    echo "Running real robot"
    ros2 action send_goal /execute_exp ras_interfaces/action/ExecuteExp {}
}

# Register in bash configuration
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
            run_real_robot
            ;;
        -h|--help)
            echo "Usage: ras_cli <command> [options]"
            echo "Commands:"
            echo "  load_experiment <exp_number>   Load the specified experiment"
            echo "  run_sim_robot                  Start the simulation robot"
            echo "  run_real_robot                 Start the real robot"
            ;;
        *)
            echo "Unknown command: $command"
            echo "Use -h or --help for usage information."
            ;;
    esac
}

# Enable bash completion for ras_cli commands
_ras_cli_completion() {
    local cur prev commands
    COMPREPLY=()   # Array of completions
    cur="${COMP_WORDS[COMP_CWORD]}"  # Current word being typed
    prev="${COMP_WORDS[COMP_CWORD-1]}"  # Previous word

    commands="load_experiment run_sim_robot run_real_robot"

    # If the first argument is 'ras_cli', suggest 'load_experiment', 'run_sim_robot', and 'run_real_robot'
    if [[ ${COMP_WORDS[0]} == "ras_cli" ]]; then
        COMPREPLY=( $(compgen -W "$commands" -- "$cur") )
    elif [[ $prev == "load_experiment" ]]; then
        # If the previous word was 'load_experiment', suggest experiment numbers (e.g., 1, 2, 3, etc.)
        COMPREPLY=( $(compgen -W "1 2 3 4 5" -- "$cur") )
    fi
}

# Register the completion function for 'ras_cli'
complete -F _ras_cli_completion ras_cli