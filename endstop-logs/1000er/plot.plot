set term svg
set output "test.svg"

set logscale xy
set xrange[1:100]

plot \
'1-count' using (abs($1-1000)):($2/4368) with histeps title '1' ,\
'2-count' using (abs($1-1000)):($2/4368) with histeps title '2' ,\
'3-count' using (abs($1-1000)):($2/4368) with histeps title '3' ,\
