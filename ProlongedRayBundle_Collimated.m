classdef ProlongedRayBundle_Collimated < RayBundle
    % extends a ray bundle beyond its end points
    % location is linearly extended along terminal tangents
    % direction is extended as parallel rays
    properties
        rb_ % the ray bundle to be prolonged
        p0_ % point at umin
        d0_ % normal at im
        t0_ % tangent at umin
        k0_ % k at umin
        p1_ % ditto for umax
        d1_
        t1_
        k1_
        du0_ % extension beyond umin, > 0
        du1_ % extension beyond umax, > 0
    end
    methods
        function obj = ProlongedRayBundle_Collimated(rb, du0, du1)
            obj.rb_ = copy(rb);
            obj.p0_ = rb.Loc(rb.umin_);
            obj.d0_ = rb.Normal(rb.umin_);
            obj.t0_ = rb.Tangent(rb.umin_);
            obj.k0_ = rb.iDir(rb.umin_);
            obj.p1_ = rb.Loc(rb.umax_);
            obj.d1_ = rb.Normal(rb.umax_);
            obj.t1_ = rb.Tangent(rb.umax_);
            obj.k1_ = rb.iDir(rb.umax_);
            obj.du0_ = du0;
            obj.du1_ = du1;
            obj.SetRange(rb.umin_ - du0, rb.umax_ + du1);
            obj.reverse_ = rb.reverse_;
        end
        
        function r = Ray(obj, u)
            for i = 1:length(u)
                uu = u(i);
                if uu >= obj.rb_.umin_ && uu <= obj.rb_.umax_
                    r(i) = obj.rb_.Ray(uu);
                else
                    r(i) = Ray(obj.Loc(uu), obj.Dir(uu));
                end
            end
        end
        
        function p = Loc(obj, u)
            obj.CheckBounds(u);
            p = nan(2, length(u));
            for i = 1:length(u)
                uu = u(i);
                if uu >= obj.rb_.umin_ && uu <= obj.rb_.umax_
                    p(:,i) = obj.rb_.Loc(uu);
                elseif uu < obj.rb_.umin_
                    p(:,i) = obj.p0_ + (uu-obj.rb_.umin_) * obj.t0_;
                else % u > obj.rb_.umax_
                    p(:,i) = obj.p1_ + (uu-obj.rb_.umax_) * obj.t1_;
                end
            end
        end
        
        function d = Normal(obj, u)
            obj.CheckBounds(u)
            d = nan(2, length(u));
            for i = 1:length(u)
                uu = u(i);
                if uu >= obj.rb_.umin_ && uu <= obj.rb_.umax_
                    d(:,i) = obj.rb_.Normal(uu);
                elseif uu < obj.rb_.umin_
                    d(:,i) = obj.d0_;
                else % u > obj.rb_.umax_
                    d(:,i) = obj.d1_;
                end
            end
        end
        
        function t = Tangent(obj, u)
            obj.CheckBounds(u)
            t = nan(2, length(u));
            for i = 1:length(u)
                uu = u(i);
                if uu >= obj.rb_.umin_ && uu <= obj.rb_.umax_
                    t(:,i) = obj.rb_.Tangent(uu);
                elseif uu < obj.rb_.umin_
                    t(:,i) = obj.t0_;
                else % u > obj.rb_.umax_
                    t(:,i) = obj.t1_;
                end
            end
        end
        
        function k = iDir(obj, u) % unreversed Dir
            obj.CheckBounds(u)
            k = nan(2, length(u));
            for i = 1:length(u)
                uu = u(i);
                if uu >= obj.rb_.umin_ && uu <= obj.rb_.umax_
                    k(:,i) = obj.rb_.iDir(uu);
                elseif uu < obj.rb_.umin_
                    k(:,i) = obj.k0_;
                else % u > obj.rb_.umax_
                    k(:,i) = obj.k1_;
                end
            end
        end
        
        function dk = idkdu(obj, u) % unreversed dkdu
            obj.CheckBounds(u)
            dk = nan(2, length(u));
            for i = 1:length(u)
                uu = u(i);
                if uu >= obj.rb_.umin_ && uu <= obj.rb_.umax_
                    dk = obj.rb_.idkdu(uu);
                else 
                    dk(:,i) = [0;0];
                end
            end
        end
    end
    methods (Static)
        function Test()
            rb = SphericalWavefrontRayBundle([1;0], 2, 0, pi/2, 1.5);
            rb.SetRange(-2,2)
            rbp = ProlongedRayBundle(rb, 0.3, 0.5);
            figure(1);
            clf;
            PlotRayBundle(gca, rb, 0.1);
            PlotRayBundle(gca, rbp, 0.2);
            rbp.Reverse();
            PlotRayBundle(gca, rbp, 0.05);
            axis equal;
        end
    end
end