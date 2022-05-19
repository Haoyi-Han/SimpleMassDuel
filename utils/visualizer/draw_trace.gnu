# GNUPlot script as a visualizer of mass point trajectory.

reset session

set title "Trajectory of Mass"
set xlabel "x"
set ylabel "y"
set xrange[-20:120]
set yrange[-20:120]

set label 1 "Origin" at 0.0,0.0 point pointtype 7 offset -3.0,-1.0

set object circle at first 39.0,41.0 radius char 5.0 \
	fillcolor rgb "red" fillstyle solid noborder
set label 2 "Hinder" at 39.0,41.0 point pointtype 7 offset -3.0,2.5

set object circle at first 100.0,100.0 radius char 0.5 \
	fillcolor rgb "blue" fillstyle solid noborder
set label 3 "Target" at 100.0,100.0 point pointtype 0 offset -3.0,1.0

plot "trace_mass.dat" using 1:2 with lines title "Mass", \
	"trace_mass.dat" using 1:2:3:4 with vectors title "Mass Vector"
