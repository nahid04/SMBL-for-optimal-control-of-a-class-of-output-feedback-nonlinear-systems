%Inverse Barrier Transformation function

function s = invbarrier(x,a,A)

s = a*A*((exp(x)-1)/(a*exp(x)-A));

end 
