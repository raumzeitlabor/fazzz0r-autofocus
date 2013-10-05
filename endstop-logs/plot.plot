set term svg
set output "test.svg"

set logscale xy


plot \
'1-count' using (abs($1-300)):($2/8765) with histeps title '1' ,\
'2-count' using (abs($1-300)):($2/8765) with histeps title '2' ,\
'3-count' using (abs($1-300)):($2/8765) with histeps title '3' ,\
