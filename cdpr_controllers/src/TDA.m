rosinit('http://localhost:11311')
r=rosrate(100);
reset(r)
sub = rossubscriber('/barycenter')
msg = receive(sub,1)

rosshutdown

