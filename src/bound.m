function [x_bounded, in_bound] = bound(x, lower_bound, upper_bound)
% Takes x: Value
%       lower_bound: Lower bound
%       upper_bound: Upper bound
% 
% Returns x_bound: Bounded value
%         in_bound: if x within bounds

    x_bounded = max(lower_bound, min(x, upper_bound)); 
    in_bound = (x >= lower_bound) && (x <= upper_bound);
end