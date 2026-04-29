alias xb="exec bash"

# Custom ls aliases
alias lla="ls -laF"

# Edit bashrc and bash_aliases
alias cb="code ~/.bashrc"
alias vb="vim ~/.bashrc"
alias nb="nano ~/.bashrc"
alias cba="code ~/.bash_aliases"
alias vba="vim ~/.bash_aliases"
alias nba="nano ~/.bash_aliases"

# Navigation shortcuts
alias cdd="cd ~/Desktop/"
alias cdudev="cd /etc/udev/rules.d/"
alias cdnetplan="cd /etc/netplan/"
alias cdxorg="cd /etc/X11/xorg.conf.d/"
alias cdws="cd ~/2026-RTX-AVC-UTA-UGV; sra"
alias cdcws="cdws; code ."

# ROS specific
alias fixroute="sudo ip route add 224.0.0.0/4 dev wlp4s0"
alias restartros2="ros2 daemon stop && ros2 daemon start && ros2 topic list"
alias sros2="source /opt/ros/jazzy/setup.sh"
alias sros2w="source install/setup.bash"
alias sra="sros2; sros2w"
alias cbsi="colcon build --symlink-install; sros2w"
alias rtl="ros2 topic list"
alias rtlv="rtl -v"
alias rtlt="rtl -t"
alias rte="ros2 topic echo"
alias rnl="ros2 node list"
alias rv=". .venv/bin/activate"

# Debug logging
alias debug='trap verbose_alias debug' # ON
alias nodebug='trap - debug' # OFF

# Other
alias lstty="ls /dev/tty*" # Show devices recognized by system
alias re-udev="sudo udevadm control --reload-rules; sudo udevadm trigger" # Reload udev rules
alias py="python3"
alias gs="git status"
# alias py3="python3"
alias avenv=". .venv/bin/activate"

alias h=history
alias c=clear
alias s=sudo
alias sa="s apt"
alias sai="sa install"
alias sar="sa remove"
alias sysinfo="landscape-sysinfo"
alias sysinfov="run-parts /etc/update-motd.d/"
alias sus="s ufw status"
alias fw="sus verbose"
alias sue="s ufw enable"
alias sud="s ufw disable"
alias sshn="sudo shutdown -h now"

# Functions
lt() {
    trap log_command debug
    # If the first argument is a number, use it as the depth level.
    # Otherwise, default to depth level 1.
    if [[ "$1" =~ ^[0-9]+$ ]]; then
        tree -LF $@
    else
        tree -LF 1 $@
    fi
    trap - debug
}

lta() {
    trap log_command debug
    # Always show all files, including hidden ones.
    # Carry over any additional arguments into lt function.
    lt $@ -aF
    trap - debug
}

log_command() {
    # Check if the command is not part of the conditional [[ ]]
    if [[ "$BASH_COMMAND" != *"[["* && "$BASH_COMMAND" != *"trap - debug"* ]]; then
        echo "Executing: $BASH_COMMAND"
    fi
}

verbose_alias() {
    local last_cmd="$(history | tail -n 1)"
    # echo $last_cmd
    # type cmd ignores the num before history.
    local cmd_type="$(type -t $last_cmd)"
    # echo $cmd_type
    if [[ $cmd_type == "alias" ]]
        then echo "-->" $BASH_COMMAND
    fi
}

# All auto command execution has been moved to .bashrc to ensure it runs in all interactive shells, including those that don't source .bash_aliases.
# This file is now solely for defining aliases and functions, while .bashrc handles the execution of commands on shell startup.
