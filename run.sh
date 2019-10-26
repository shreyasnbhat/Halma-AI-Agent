n=0;
while [[ $n -lt 300 ]];
do
  ./a.out
  python runner.py
  n=$((n+1));
done