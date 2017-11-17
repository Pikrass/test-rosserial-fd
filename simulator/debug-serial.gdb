set $recv=0
set pagination off

break node_handle.h:210
commands
	silent
	if $recv
		printf " %02hx", data
	else
		printf "\nRecv %02hx", data
	end
	set $recv=1
	cont
	end

break ArduinoHardware::write
commands
	silent
	printf "\nSend"
	set $recv=0
	cont
	end

break ArduinoHardware.h:102
commands
	silent
	printf " %02hx", data[i]
	cont
	end
