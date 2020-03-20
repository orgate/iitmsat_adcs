function [q]=quat_inv(s)
q=[-s(1:3);s(4)];
end