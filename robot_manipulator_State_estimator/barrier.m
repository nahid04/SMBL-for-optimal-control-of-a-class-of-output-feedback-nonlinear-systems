%Function to transform from x - coordinate to s- coordinate

function s = barrier(x,a,A)

s = log((A*(a-x))./(a.*(A-x)));

end 
