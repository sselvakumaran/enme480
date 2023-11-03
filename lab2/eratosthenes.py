def sieve(n):
  l = list(range(2,n + 1))
  i = 0
  p = l[i]
  while p < n / 2:
    l = list(filter(lambda x: x == p or x % p != 0, l))
    i += 1
    p = l[i]
  print(l)
  print(f'Number of primes: {len(l)}')
sieve(1000)