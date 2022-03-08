from .hma_eval import *
from .data import hma_config


class YolactDetector:
    def __init__(self, use_gpu, config, weight, th, top_k):
        args = parse_args("")

        args.cuda = use_gpu
        args.config = config
        args.trained_model = weight
        args.score_threshold = th
        args.top_k = int(top_k)
        args.display_scores = False
        args.display_masks = False

        self.args = args
        self.net = None

        import warnings
        warnings.filterwarnings("ignore")

        from utils.logging_helper import setup_logger
        self.logger = setup_logger("yolact")


    def setup(self):
        global cfg
        cfg = hma_config.set_hma_cfg(self.args.config)

        with torch.no_grad():
            if self.args.cuda:
                cudnn.benchmark = True
                cudnn.fastest = True
                if self.args.deterministic:
                    cudnn.deterministic = True
                    cudnn.benchmark = False
                torch.set_default_tensor_type("torch.cuda.FloatTensor")
            else:
                torch.set_default_tensor_type("torch.FloatTensor")

            self.logger.info("Loading model...")
            self.logger.info(str(self.args.trained_model))
            self.net = Yolact(training=False)
            self.net.load_weights(self.args.trained_model, args=self.args)
            self.net.eval()
            self.logger.info("Model loaded.")

            if self.args.cuda:
                self.net = self.net.cuda()

    def detect(self, cv_rgb):
        self.net.detect.use_fast_nms = self.args.fast_nms
        cfg.mask_proto_debug = False

        frame = torch.from_numpy(cv_rgb).cuda().float()
        batch = FastBaseTransform()(frame.unsqueeze(0))

        extras = {"backbone": "full", "interrupt": False, "keep_statistics": False, "moving_statistics": None}
        preds = self.net(batch, extras=extras)["pred_outs"]

        return prep_display(preds, frame, None, None, undo_transform=False, mask_alpha=0.5)