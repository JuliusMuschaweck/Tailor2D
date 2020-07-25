function PlotRayBundle( ax, rb, kLength, varargin)
    uu = linspace(rb.umin_,rb.umax_,21);
    iloc = rb.Loc(uu);
    ik = rb.Dir(uu);
    plot(ax, iloc(1,:),iloc(2,:),'k');
    hold on;
    if kLength > 0
        iloc2 = iloc + kLength*ik;
        for i = 1:length(uu)
            plot([iloc(1,i),iloc2(1,i)],[iloc(2,i),iloc2(2,i)], varargin{:});
        end
    end
end
