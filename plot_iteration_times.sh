#! /bin/bash

# gnuplot -e 'set terminal pdf; set output "iteration_times.pdf"; set ylabel "seconds"; set xlabel "iteration"; plot "iteration_times.dat" with boxes'
gnuplot -e 'set terminal pdf; set output "ball-approach_iteration_times.pdf"; set ylabel "seconds"; set xlabel "iteration"; plot "ball-approach_iteration_times.dat" with linespoints lt 1 pt 7 ps 0.5'
