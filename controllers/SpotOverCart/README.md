# Spot Tries To Get Off a Cart
Drop Spot on top of a cart, and make it escape from it


## Failed jump
Tried to stand up really quickly from a sitting position, but the motor seemed not powerful enough.

![Spot Trying](../../docs/spot_trying.gif)


## Baby crawl
Tried heuristics with the joints, and it walked forward! Ok, "walked" is a bit of a stretch, it crawled forward, seemingly in huge pain. Again the elbow motor seemed too weak.

![Spot Crawling](../../docs/spot_crawling.gif)

## Walked
Walked at last! But with hand picked params. Would be good to be able to run Webots simulations programmatically, to supply various params on multiple runs and do gradient descent, rewarding furthest distance or speed.

![Spot Walking](../../docs/spot_walking.gif)

## Challenge
* Create algo to walk away from the cart (which moves as it has wheels)
