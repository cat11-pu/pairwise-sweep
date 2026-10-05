# sweep

二维轴对齐包围盒（AABB）的扫掠碰撞查询与分步积分，供平台跳跃类游戏做角色位移。

## 目录

- `sweep/core.py` —— 全部碰撞与积分实现
- `tests/test_core.py` —— unittest 用例

## 运行测试

在项目根目录执行：

    python3 -m unittest discover -s tests -v

不需要任何第三方依赖，标准库 Python 3 即可。

## 公开接口

- `AABB(cx, cy, hw, hh)`：中心点加半宽半高，带 `left` / `right` / `bottom` / `top` / `width` / `height`
- `AABB.overlaps(other)`：两个矩形的内部是否共享空间，仅仅接触不算重叠
- `AABB.moved(dx, dy)`：平移后的副本
- `AABB.expanded(amount)`：四周外扩后的副本，用于范围更大的查询
- `aabb_from_ltrb(left, bottom, right, top)`：按四条边构造矩形
- `axis_times(lo, hi, d, olo, ohi)`：一维移动区间与固定区间共享空间的比例区间
- `sweep_aabb(box, dx, dy, obstacles)`：单步内最早接触，返回 `Hit` 或 `None`
- `substep_count(box, dx, dy, obstacles, max_step)`：一次移动要拆成多少个子步
- `move_and_collide(box, dx, dy, obstacles, max_step)`：分子步移动并与障碍物分离，返回 `(box, hits)`
