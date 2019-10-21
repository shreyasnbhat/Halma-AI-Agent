n=0;
while [[ $n -lt 250 ]];
do
  ./a.out
  python runner.py
  n=$((n+1));
done