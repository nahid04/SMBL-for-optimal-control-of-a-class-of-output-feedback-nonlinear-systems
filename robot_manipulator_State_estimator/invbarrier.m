%Function to transform from s - coordinate to x- coordinate

function s = invbarrier(x,a,A)

s = a*A*((exp(x)-1)/(a*exp(x)-A));

end 
