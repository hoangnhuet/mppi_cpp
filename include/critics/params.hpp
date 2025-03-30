namespace mppi::critics
{
    class Params
    {
        public:
            Params();
            ~Params();
            float power_{1};
            float weight_{1};
            float v_max = 0.5;
            float v_min = -0.5;
    };
}