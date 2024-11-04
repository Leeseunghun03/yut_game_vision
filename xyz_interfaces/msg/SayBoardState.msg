bool start_game
bool in_game
bool end_game

bool turn # 0이면 상대턴
bool yut_throw_flag # 내 턴에 윷을 던질 때 게임 결과에 따른 답변생성
bool my_turn_res_flag # 게임 말을 옮긴 후 내 턴을 마칠 때 결과에 따른 답변 생성

bool button_pushed # 게임 도중 버튼을 눌러 대화를 시도할 때 flag

int32[] positions_p1 # 음수 개수만 비교
int32[] positions_p2 # 음수 개수만 비교
int32 robotgoaltokens
int32 playergoaltokens
