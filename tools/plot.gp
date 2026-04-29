if (!exists("filename")) filename="output.csv"

set datafile separator ","

# Use as title the current headers
set key autotitle columnhead

set title "System Response"
set xlabel "Time"
set ylabel "Value"

plot filename using 1:2 with lines, \
     filename using 1:3 with lines

pause -1

set title "Control Signal"
set xlabel "Time"
set ylabel "u(t)"

plot filename using 1:4 with lines

pause -1
