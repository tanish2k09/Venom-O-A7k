#ifndef _LINUX_KLAPSE_H
#define _LINUX_KLAPSE_H

/* Required variables for external access. Change as per use */
extern void set_rgb_slider(int bl_lvl);

extern void update_rgb(int force_r, int force_g, int force_b);

#define K_RED    mtk_disp_ld_r
#define K_GREEN  mtk_disp_ld_g
#define K_BLUE   mtk_disp_ld_b

#define K_TYPE   u32

extern K_TYPE K_RED, K_GREEN, K_BLUE;

#endif  /* _LINUX_KLAPSE_H */
