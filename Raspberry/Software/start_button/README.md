# Button logic

* Controller script is started on boot. This can be enabled or disabled by corresponding scripts. Just run them with ./name.

* Crontab checks every 5 seconds, if the controller script is running. If it isn't, it starts it.

* Currently running controller code can be seen by "screen -x controller"
