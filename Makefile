simple:
	g++ simpleMotionCommands.cpp -o $@ -I/usr/local/Aria/include -L/usr/local/Aria/lib -lAria
action:
	g++ actionExample.cpp -o $@ -I/usr/local/Aria/include -L/usr/local/Aria/lib -lAria
carto:
	g++ carto.cpp -o $@ -I/usr/local/Aria/include -L/usr/local/Aria/lib -lAria
laser:
	g++ laser.cpp -o $@ -I/usr/local/Aria/include -L/usr/local/Aria/lib -lAria

