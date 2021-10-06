%Barrier Transformation function

function s = barrier(x,a,A)

s = log((A*(a-x))./(a.*(A-x)));

end 
