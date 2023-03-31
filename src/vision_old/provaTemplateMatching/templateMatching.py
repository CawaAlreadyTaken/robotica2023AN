#/usr/bin/env python3
import cv2

blocco = cv2.imread('blocco.jpg')
blocco_grigio = cv2.cvtColor(blocco, cv2.COLOR_BGR2GRAY)

template_alto = cv2.imread('alto.jpg', 0)
template_laterale = cv2.imread('laterale.jpg', 0)
template_basso = cv2.imread('basso.jpg', 0)

template_laterale = cv2.resize(template_laterale, (int(900*1.2), int(1267*1.7)))
cv2.imshow('alto', template_laterale)
cv2.waitKey(0)


print(blocco_grigio.shape)

print()

print(template_alto.shape)
print(template_laterale.shape)
print(template_basso.shape)

result_alto = cv2.matchTemplate(blocco_grigio, template_alto, cv2.TM_CCOEFF_NORMED)
result_laterale = cv2.matchTemplate(blocco_grigio, template_laterale, cv2.TM_CCOEFF_NORMED)
result_basso = cv2.matchTemplate(blocco_grigio, template_basso, cv2.TM_CCOEFF_NORMED)

min_val_alto, max_val_alto, min_loc_alto, max_loc_alto = cv2.minMaxLoc(result_alto)
min_val_laterale, max_val_laterale, min_loc_laterale, max_loc_laterale = cv2.minMaxLoc(result_laterale)
min_val_basso, max_val_basso, min_loc_basso, max_loc_basso = cv2.minMaxLoc(result_basso)

print(max_val_alto, max_val_basso, max_val_laterale)
if max_val_alto > max_val_laterale and max_val_alto > max_val_basso:
    print("Il blocco è orientato verso l'alto.")
elif max_val_laterale > max_val_alto and max_val_laterale > max_val_basso:
    print("Il blocco è orientato lateralmente.")
else:
    print("Il blocco è orientato verso il basso.")
