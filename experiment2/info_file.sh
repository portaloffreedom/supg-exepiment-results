
for filename in $@; do
  echo -n "$filename: "
  grep "Evaluating gait, fitness =" $filename |cut -c 28- |wc -l
  echo -n "best value = "
  grep "Evaluating gait, fitness =" $filename |cut -c 28- |sort -n -r |grep -v 'e-' |head -n1  
done

