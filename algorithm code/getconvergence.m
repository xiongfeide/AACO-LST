function convergence_num=getconvergence(len)
    n=length(len);
    for i=1:n
        if(len(i)==len(n))
            convergence_num=i;
            return;
        end
    end
end