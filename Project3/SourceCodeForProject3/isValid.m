function [validation] = isValid(position, d)
            if d(position(2), position(1)) == 0
                validation = true;
            else
                validation = false;
            end
end