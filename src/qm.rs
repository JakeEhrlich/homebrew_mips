//! Quine-McCluskey two-level minimisation, for turning truth tables into
//! product terms that fit a 22V10 macrocell.  Fine up to a dozen inputs.

/// A product term over inputs `0..n`: `(input, positive)` literals.
pub type Cube = Vec<(usize, bool)>;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
struct Imp {
    /// Bits that are fixed (1 = literal present).
    mask: u32,
    /// Values of the fixed bits.
    val: u32,
}

/// Minimal-ish sum of products for `f` over `n` inputs.  `f(m)` returns
/// `Some(true)` for minterms, `Some(false)` for maxterms, `None` for don't
/// cares (input bit `i` of `m` is input `i`).
pub fn minimize(n: usize, f: impl Fn(u32) -> Option<bool>) -> Vec<Cube> {
    assert!(n <= 16, "too many inputs for exhaustive minimisation");
    let full = (1u32 << n) - 1;
    let mut ones = Vec::new();
    let mut cares = Vec::new();
    for m in 0..(1u32 << n) {
        match f(m) {
            Some(true) => {
                ones.push(m);
                cares.push(m);
            }
            None => cares.push(m),
            Some(false) => {}
        }
    }
    if ones.is_empty() {
        return vec![];
    }
    if ones.len() as u32 == (1u32 << n) {
        return vec![vec![]]; // constant true
    }

    // Prime implicants by repeated pairwise combination, grouped by popcount.
    let mut current: Vec<Imp> = cares.iter().map(|&m| Imp { mask: full, val: m }).collect();
    current.sort();
    current.dedup();
    let mut primes: Vec<Imp> = Vec::new();
    loop {
        let mut next: Vec<Imp> = Vec::new();
        let mut used = vec![false; current.len()];
        // Index by (mask, popcount): only groups one apart can combine.
        let mut groups: std::collections::BTreeMap<(u32, u32), Vec<usize>> = Default::default();
        for (i, imp) in current.iter().enumerate() {
            groups.entry((imp.mask, imp.val.count_ones())).or_default().push(i);
        }
        for (&(mask, pop), lo) in &groups {
            let Some(hi) = groups.get(&(mask, pop + 1)) else { continue };
            for &i in lo {
                for &j in hi {
                    let (a, b) = (current[i], current[j]);
                    let diff = a.val ^ b.val;
                    if diff.count_ones() == 1 {
                        used[i] = true;
                        used[j] = true;
                        next.push(Imp { mask: a.mask & !diff, val: a.val & !diff });
                    }
                }
            }
        }
        for (i, imp) in current.iter().enumerate() {
            if !used[i] {
                primes.push(*imp);
            }
        }
        if next.is_empty() {
            break;
        }
        next.sort();
        next.dedup();
        current = next;
    }
    primes.sort();
    primes.dedup();

    // Cover the ones (don't cares need no cover).
    let covers = |imp: &Imp, m: u32| (m & imp.mask) == imp.val;
    let mut chosen: Vec<Imp> = Vec::new();
    let mut uncovered: Vec<u32> = ones.clone();
    // Essential primes first.
    for &m in &ones {
        let c: Vec<&Imp> = primes.iter().filter(|p| covers(p, m)).collect();
        if c.len() == 1 && !chosen.contains(c[0]) {
            chosen.push(*c[0]);
        }
    }
    uncovered.retain(|&m| !chosen.iter().any(|p| covers(p, m)));
    // Then greedy by coverage, ties broken by fewer literals.
    while !uncovered.is_empty() {
        let best = primes
            .iter()
            .filter(|p| !chosen.contains(p))
            .max_by_key(|p| (uncovered.iter().filter(|&&m| covers(p, m)).count(), 32 - p.mask.count_ones()))
            .unwrap();
        let best = *best;
        chosen.push(best);
        uncovered.retain(|&m| !covers(&best, m));
    }
    chosen
        .iter()
        .map(|p| (0..n).filter(|&i| p.mask >> i & 1 == 1).map(|i| (i, p.val >> i & 1 == 1)).collect())
        .collect()
}

/// Evaluate a sum of products on an input vector.
pub fn eval(terms: &[Cube], m: u32) -> bool {
    terms.iter().any(|t| t.iter().all(|&(i, pos)| (m >> i & 1 == 1) == pos))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check(n: usize, f: impl Fn(u32) -> bool + Copy) -> Vec<Cube> {
        let terms = minimize(n, |m| Some(f(m)));
        for m in 0..(1u32 << n) {
            assert_eq!(eval(&terms, m), f(m), "m={m:b} terms={terms:?}");
        }
        terms
    }

    #[test]
    fn xor_and_majority() {
        assert_eq!(check(2, |m| (m & 1 == 1) != (m >> 1 & 1 == 1)).len(), 2);
        // Majority of 3: a&b + a&c + b&c.
        let t = check(3, |m| m.count_ones() >= 2);
        assert_eq!(t.len(), 3);
        assert!(t.iter().all(|c| c.len() == 2));
        // 3-input XOR: 4 minterms, none combine.
        assert_eq!(check(3, |m| m.count_ones() % 2 == 1).len(), 4);
    }

    #[test]
    fn dont_cares_reduce_terms() {
        // f = a for even m, don't care otherwise... f(m) = bit0 when bit1 == 0, dc when bit1 == 1.
        let t = minimize(2, |m| if m >> 1 & 1 == 1 { None } else { Some(m & 1 == 1) });
        assert_eq!(t, vec![vec![(0, true)]]);
    }

    #[test]
    fn constants() {
        assert_eq!(minimize(3, |_| Some(false)), Vec::<Cube>::new());
        assert_eq!(minimize(3, |_| Some(true)), vec![Vec::<(usize, bool)>::new()]);
    }

    /// The 3-bit carry-select sums from the ALU plan: s2 with carry-in 0
    /// needs 16 terms, and no more.
    #[test]
    fn adder_slice_sizes() {
        // inputs: a0 a1 a2 b0 b1 b2
        let sum = |m: u32, cin: u32, bit: usize| {
            let a = m & 7;
            let b = m >> 3 & 7;
            ((a + b + cin) >> bit) & 1 == 1
        };
        assert_eq!(check(6, |m| sum(m, 0, 2)).len(), 16);
        assert_eq!(check(6, |m| sum(m, 1, 2)).len(), 16);
        assert!(check(6, |m| sum(m, 0, 1)).len() <= 6);
        assert!(check(6, |m| sum(m, 1, 1)).len() <= 6);
        // carry out for both carry-ins (the second is G+P, not the POS
        // "all propagate" form, and costs more terms)
        assert!(check(6, |m| ((m & 7) + (m >> 3 & 7)) >> 3 & 1 == 1).len() <= 7);
        assert!(check(6, |m| ((m & 7) + (m >> 3 & 7) + 1) >> 3 & 1 == 1).len() <= 14);
    }
}
