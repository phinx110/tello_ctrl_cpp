class SuperState {
   public:
    virtual std::unique_ptr<SuperState> next_state() =0;
};

class state_rest : public SuperState {
public:
    std::unique_ptr<SuperState> next_state() override;
};

class state_liftoff : public SuperState {
    std::unique_ptr<SuperState> next_state() override;
};

class state_steady : public SuperState {
    std::unique_ptr<SuperState> next_state() override;
};

class state_land : public SuperState {
    std::unique_ptr<SuperState> next_state() override;
};

class search : public SuperState {
    std::unique_ptr<SuperState> next_state() override;
};
