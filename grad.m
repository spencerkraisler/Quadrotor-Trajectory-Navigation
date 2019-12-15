% numerical differentiation of gradients
function fn = grad(f,n,ds)
    dx = [ds 0 0]'; dy = [0 ds 0]'; dz = [0 0 ds]';
    fx = (f(n + dx) - f(n - dx))/(2*ds);
    fy = (f(n + dy) - f(n - dy))/(2*ds);
    fz = (f(n + dz) - f(n - dz))/(2*ds);
    fn = [fx fy fz]';
end