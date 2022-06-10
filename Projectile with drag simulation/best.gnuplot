set datafile separator ","
plot 'best.csv' using 1:2 title "angle" pt 7 ps 0.1, \
'best.csv' using 1:3 title "velocity" pt 7 ps 0.1