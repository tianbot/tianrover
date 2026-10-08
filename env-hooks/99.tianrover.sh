# Deployment configuration is outside the repository and is read on each source.
_tianrover_config="${TIANROVER_CONFIG_FILE:-${HOME}/.config/tianrover/my_tianrover.sh}"
if [ -r "$_tianrover_config" ]; then
  . "$_tianrover_config"
fi
unset _tianrover_config
