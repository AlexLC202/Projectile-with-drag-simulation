set datafile separator ","
set size ratio -1
plot 'projectile.csv' using 1:2 title "position" pt 7 ps 0.1, \
'projectile.csv' using 3:4 title "best shot" pt 7 ps 0.1