function value = jh_smoothstep(x)
clamped = min(max(x, 0.0), 1.0);
value = clamped .* clamped .* (3.0 - 2.0 .* clamped);
end
