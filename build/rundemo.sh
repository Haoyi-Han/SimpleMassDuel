echo "start program for ticktime=1.0 ..."
./demo -t -c 1.0 -r bt_trace_1.0 -s trace_mass_1.0.dat -l > log_1.0.log
echo "start program for ticktime=0.75 ..."
./demo -t -c 0.75 -r bt_trace_0.75 -s trace_mass_0.75.dat -l > log_0.75.log
echo "start program for ticktime=0.5 ..."
./demo -t -c 0.5 -r bt_trace_0.5 -s trace_mass_0.5.dat -l > log_0.5.log
echo "start program for ticktime=0.25 ..."
./demo -t -c 0.25 -r bt_trace_0.25 -s trace_mass_0.25.dat -l > log_0.25.log
echo "start program for ticktime=0.125 ..."
./demo -t -c 0.125 -r bt_trace_0.125 -s trace_mass_0.125.dat -l > log_0.125.log
echo "start program for ticktime=0.05 ..."
./demo -t -c 0.05 -r bt_trace_0.05 -s trace_mass_0.05.dat -l > log_0.05.log
echo "all programs finished."
