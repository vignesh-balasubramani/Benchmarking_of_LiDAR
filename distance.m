function [mindist,maxdist]= distance(u1,u2,ttc12)
%function that calculates the maximum and minimum distance for given ranges of TTC and velocities
    d=zeros(length(u1)*length(u2)*length(ttc12),1);
    l=1;
    for i=1:length(u1)
        for j=1:length(u2)
            for k=1:length(ttc12)
                d(l)=ttc12(k)*abs(u1(i)-u2(j));
                l=l+1;            
            end
        end
    end
    maxdist=max(d);
    mindist=min(d(d>0));
end
