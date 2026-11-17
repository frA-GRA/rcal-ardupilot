// Re-emit rcal's schema version cfgs (e.g. `uci_ge_2_5`) so this crate can match the
// schema rcal was built with.
fn main() {
    let (Ok(cfgs), Ok(check)) = (
        std::env::var("DEP_RCAL_CFGS"),
        std::env::var("DEP_RCAL_CHECK_CFGS"),
    ) else {
        println!("cargo::warning=rcal exports no schema version cfgs; assuming UCI < 2.5");
        println!("cargo::rustc-check-cfg=cfg(uci_ge_2_5)");
        return;
    };
    for c in check.split(';').filter(|c| !c.is_empty()) {
        println!("cargo::rustc-check-cfg={c}");
    }
    for c in cfgs.split(';').filter(|c| !c.is_empty()) {
        println!("cargo::rustc-cfg={c}");
    }
}
