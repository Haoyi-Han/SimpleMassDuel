# GNUPlot script as a visualizer of mass point trajectory.

reset session

set title "Trajectory of Mass"
set xlabel "x"
set ylabel "y"
set xrange[-10:110]
set yrange[-10:110]
set key top left

set label 1 "Origin" at 0.0,0.0 point pointtype 7 offset -3.0,-1.0
set label 2 "Target" at 100.0,100.0 point pointtype 7 offset -3.0,1.0
set label 3 "Hinder-1" at 39.0,41.0 point pointtype 7 offset -4.5,2.5
set label 4 "Hinder-2" at 61.0,61.0 point pointtype 7 offset -4.5,2.5
set label 5 "Hinder-3" at 80.0,75.0 point pointtype 7 offset -4.5,2.5

plot "point_list.dat" using 1:2:($3*2):($3*2):(0):4 with ellipses title "Point"  \
	fillcolor rgbcolor variable fillstyle solid noborder, \
	"trace_mass.dat" using 1:2 with lines title "Mass", \
	"trace_mass.dat" using 1:2:3:4 with vectors title "Mass Vector"
