function d = Inflate(obsPos, inflateThichness, d)
    for i = -inflateThichness : inflateThichness
        for j = -inflateThichness : inflateThichness
            if norm([i, j]) <= inflateThichness
                if d(obsPos(1) + i, obsPos(2) + j) ~= 100
                    d(obsPos(1) + i, obsPos(2) + j) = 99;
                end
            end
        end
    end
end