cat screenlog.0 | awk '{print $4}' | hex2dec | count > 1-count
cat screenlog.0 | awk '{print $5}' | hex2dec | count > 2-count
cat screenlog.0 | awk '{print $6}' | hex2dec | count > 3-count
